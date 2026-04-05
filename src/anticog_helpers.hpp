/**
 * @file anticog_helpers.hpp
 * @brief Anticogging compensator: look-up table interpolation of pre-recorded
 *        cogging torque or phase-voltage maps.
 *
 * @details Cogging torque is caused by the magnetic reluctance variation between
 * rotor and stator teeth as the rotor turns. It appears as a periodic disturbance
 * at the electrical frequency (or its harmonics) and degrades torque smoothness at
 * low speeds. `AnticoggingCompensator` cancels it by injecting a pre-measured
 * feedforward correction indexed by shaft angle.
 *
 * ## Compensation modes
 * The compensator supports two storage formats, which can be used independently
 * or together:
 *
 * | Mode         | Constructor argument                            | Usage |
 * |--------------|------------------------------------------------|-------|
 * | Torque map   | `std::array<float, steps_>` (torque [Nm])      | Added to the controller torque request as a feedforward offset |
 * | Voltage map  | `PhaseValues<std::array<float, steps_>>` (V)   | Added directly to phase PWM duty as a voltage feedforward |
 *
 * ## Interpolation
 * Both `get_cogging_torque()` and `get_cogging_voltage()` use **linear interpolation**
 * between adjacent look-up table entries:
 * @f[
 *   y(\theta) = y[i_l]\,\alpha + y[i_u]\,\beta
 * @f]
 * where @f$ i_l = \lfloor \theta \cdot N / 2\pi \rfloor @f$,
 * @f$ i_u = i_l + 1 \pmod{N} @f$, @f$ \alpha = \lceil \text{idx} \rceil - \text{idx} @f$,
 * @f$ \beta = 1 - \alpha @f$, and @f$ N = \text{steps\_} @f$.
 *
 * @note Maps are recorded by `CoggingMapper` (see `cogging_mapper.hpp`) which runs
 *       a PD+I position-control sweep and averages the torque/voltage required to
 *       hold each angular sample point.
 */

#ifndef ANTICOG_HELPERS_HPP
#define ANTICOG_HELPERS_HPP
#include "helpers.hpp"
#include <cmath>
#include <array>
#include "transformations.hpp"

/// @brief Empty torque map sentinel (zero-length, used when no torque map is loaded).
inline std::array<float, 0> default_anticog_torque_map{};

/// @brief Empty voltage map sentinel (zero-length, used when no voltage map is loaded).
inline PhaseValues<std::array<float, 0>> default_anticog_volt_map{};


/**
 * @brief Anticogging feedforward compensator backed by compile-time-sized LUTs.
 *
 * @details Holds references to pre-recorded cogging maps (torque or per-phase
 * voltage) and provides angle-indexed linear interpolation. The LUT size
 * `steps_` is a compile-time template parameter; typical values are 200–1000
 * uniformly-spaced samples over 2π mechanical radians.
 *
 * @tparam steps_ Number of equally-spaced look-up table entries covering one
 *                full mechanical revolution [0, 2π). Must be > 0 for non-trivial
 *                compensation; passing 0 causes all query functions to return zero.
 *
 * @note The maps are stored as `const` references — the `AnticoggingCompensator`
 *       does not own the backing arrays. The caller must ensure the arrays remain
 *       alive for the compensator's lifetime (typically static or global arrays).
 */
template<std::size_t steps_>
class AnticoggingCompensator{
  public:
    ~AnticoggingCompensator() = default;

    /**
     * @brief Constructs with a torque map only (voltage map set to empty).
     * @param anticog_torque_map Reference to an array of `steps_` cogging-torque
     *                           values [Nm], uniformly indexed over [0, 2π).
     */
    AnticoggingCompensator(const std::array<float, steps_> & anticog_torque_map)
    : anticog_torque_map_(anticog_torque_map),
      anticog_volt_map_(empty_volt_map())
    {}

    /**
     * @brief Constructs with a per-phase voltage map only (torque map set to empty).
     * @param anticog_volt_map Reference to a `PhaseValues<std::array<float, steps_>>`
     *                         containing per-phase cogging voltages [V].
     */
    AnticoggingCompensator(const PhaseValues<std::array<float, steps_>> & anticog_volt_map)
    : anticog_torque_map_(empty_torque_map()),
      anticog_volt_map_(anticog_volt_map)
    {}

    /**
     * @brief Constructs with both a torque map and a voltage map.
     * @param anticog_torque_map Cogging torque map [Nm].
     * @param anticog_volt_map   Per-phase cogging voltage map [V].
     */
    AnticoggingCompensator(const std::array<float, steps_> & anticog_torque_map, PhaseValues<std::array<float, steps_>> & anticog_volt_map)
    : anticog_torque_map_(anticog_torque_map),
      anticog_volt_map_(anticog_volt_map)
    {}

  /**
   * @brief Queries the cogging torque at a given shaft angle via linear interpolation.
   *
   * @details Normalises `rads` to [0, 2π), computes the fractional LUT index,
   * and linearly interpolates between the two surrounding table entries (with
   * wrap-around at the table boundary).
   *
   * Returns 0 immediately if `steps_ == 0` (no map loaded).
   *
   * @param rads Shaft angle [rad] (any range; normalised internally).
   * @returns    Cogging torque feedforward [Nm] at the given angle.
   */
  float get_cogging_torque(const float rads) const
  {
    if(steps_ == 0){
      return 0.f;
    }
    float ang = normalize_angle(rads);
    if(ang < 0) { ang += _2_PI_; }

    const float idx  = static_cast<float>(steps_) / _2_PI_ * ang;
    const size_t idl = static_cast<size_t>(idx) % steps_;
    const size_t idu = (idl + 1) % steps_;

    const float alpha = ceilf(idx) - idx;
    const float beta  = 1.f - alpha;

    return anticog_torque_map_.at(idl) * alpha + anticog_torque_map_.at(idu) * beta;
  }

  /**
   * @brief Queries the per-phase cogging voltage at a given shaft angle via linear interpolation.
   *
   * @details Same interpolation algorithm as `get_cogging_torque()`, applied
   * independently to each of the three phase-voltage LUT arrays (a, b, c).
   *
   * Returns @f$ \{0, 0, 0\} @f$ immediately if `steps_ == 0`.
   *
   * @param rads Shaft angle [rad] (any range; normalised internally).
   * @returns    Per-phase cogging voltage feedforward [V] as `PhaseValues<float>`.
   */
  PhaseValues<float> get_cogging_voltage(const float rads) const
  {
    if(steps_ == 0){
      return {0.f, 0.f, 0.f};
    }
    float ang = normalize_angle(rads);
    if(ang < 0) { ang += _2_PI_; }

    const float  idx  = static_cast<float>(steps_) / _2_PI_ * ang;
    const size_t idl  = static_cast<size_t>(idx) % steps_;
    const size_t idu  = (idl + 1) % steps_;

    const float alpha = ceilf(idx) - idx;
    const float beta  = 1.f - alpha;

    return {anticog_volt_map_.a.at(idl) * alpha + anticog_volt_map_.a.at(idu) * beta,
            anticog_volt_map_.b.at(idl) * alpha + anticog_volt_map_.b.at(idu) * beta,
            anticog_volt_map_.c.at(idl) * alpha + anticog_volt_map_.c.at(idu) * beta};
  }

  private:
    const std::array<float, steps_> & anticog_torque_map_; ///< Cogging torque LUT reference [Nm].
    const PhaseValues<std::array<float, steps_>> & anticog_volt_map_; ///< Per-phase voltage LUT reference [V].

    /// @brief Returns a reference to a zero-filled static torque array (empty sentinel).
    static const std::array<float, steps_> & empty_torque_map() {
      static const std::array<float, steps_> empty{};
      return empty;
    }

    /// @brief Returns a reference to a zero-filled static voltage map (empty sentinel).
    static const PhaseValues<std::array<float, steps_>> & empty_volt_map() {
      static const PhaseValues<std::array<float, steps_>> empty{};
      return empty;
    }
};


#endif // ANTICOG_HELPERS_HPP
