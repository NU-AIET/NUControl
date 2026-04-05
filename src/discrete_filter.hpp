/**
 * @file discrete_filter.hpp
 * @brief General discrete-time IIR/FIR filter, Butterworth 2nd-order, PI controller,
 *        and motor feedforward filter implementations.
 *
 * @details All filters share the `DiscreteFilter<T,G>` base class which implements
 * the standard difference equation:
 * @f[
 *   y[n] = \sum_{k=0}^{N-1} b_k\, x[n-k]
 *          - \sum_{j=0}^{M-1} a_j\, y[n-j-1]
 * @f]
 * where @f$ b_k @f$ are the feedforward (input / numerator) coefficients and
 * @f$ a_j @f$ are the feedback (output / denominator) coefficients. The sign
 * convention matches standard direct-form II transposed (i.e., feedback
 * coefficients are **subtracted**).
 *
 * Specialisations:
 * - `Butterworth2nd<T>` — 2nd-order Butterworth low-pass (bilinear transform)
 * - `PIController<T>` — Tustin-discretised PI controller
 * - `MotorFeedforward` — bilinear RL plant inversion for feedforward current control
 */

#ifndef DISCRETE_FILTER_HPP
#define DISCRETE_FILTER_HPP
#include <vector>
#include <algorithm>
#include "helpers.hpp"


// ===========================================================================
// RingBuffer
// ===========================================================================

/**
 * @brief Fixed-capacity circular shift register backed by a pre-allocated vector.
 *
 * @details Replaces `std::deque` in `DiscreteFilter` shift registers to eliminate
 * heap allocation from the ISR hot path. The capacity is fixed at construction;
 * `push()` overwrites the oldest slot in O(1) without any heap operations.
 *
 * Indexing is by *age*: `[0]` is the most recently pushed value, `[1]` is one
 * sample older, etc.
 *
 * @tparam T Element type.
 */
template<typename T>
struct RingBuffer
{
  RingBuffer() = default;

  RingBuffer(size_t n, T fill = T{}) : buf_(n, fill), head_(0) {}

  /// @brief Inserts `val` as the newest element, overwriting the oldest slot.
  void push(const T& val)
  {
    if (buf_.empty()) return;
    head_ = (head_ == 0 ? buf_.size() : head_) - 1;
    buf_[head_] = val;
  }

  /// @brief Access by age. `[0]` = newest, `[1]` = one sample older, …
  T&       operator[](size_t age)       { return buf_[(head_ + age) % buf_.size()]; }
  const T& operator[](size_t age) const { return buf_[(head_ + age) % buf_.size()]; }

  /// @brief Fill all slots with `val` and reset head to 0.
  void fill_all(const T& val) { std::fill(buf_.begin(), buf_.end(), val); head_ = 0; }

  size_t size()  const { return buf_.size(); }
  bool   empty() const { return buf_.empty(); }

  std::vector<T> buf_;   ///< Storage (allocated once at construction)
  size_t         head_;  ///< Index of the most recent element
};

// ===========================================================================
// DiscreteFilter
// ===========================================================================

/**
 * @brief Generic discrete-time linear filter (IIR / FIR).
 *
 * @details Implements the difference equation:
 * @f[
 *   y[n] = \sum_{k=0}^{N-1} b_k\, x[n-k]
 *          - \sum_{j=0}^{M-1} a_j\, y[n-j-1]
 * @f]
 * Shift registers are `RingBuffer` instances (fixed-capacity, heap-allocation-free
 * after construction). For FIR filters (no feedback), pass an empty `output_coeffs`
 * vector; the output ring is then skipped entirely.
 *
 * @tparam T Type of the filter state and input/output values (e.g., `float`,
 *           `QuadDirectValues<float>`, `PhaseValues<float>`). Must support
 *           `+`, `−`, and scalar `*` with type `G`.
 * @tparam G Type of filter coefficients (typically `float`).
 *
 * @note `T` must be default-constructible as `static_cast<T>(0.f)` for state
 *       initialisation. This works for arithmetic types and for
 *       `PhaseValues<T>` / `QuadDirectValues<T>` which have scalar constructors.
 */
template<typename T, typename G>
class DiscreteFilter
{
public:
  DiscreteFilter() = default;
  ~DiscreteFilter() = default;

  /**
   * @brief Constructs the filter with given numerator and denominator coefficients.
   *
   * @param input_coeffs  Feedforward (numerator) coefficients `b_0, b_1, …, b_{N-1}`.
   *                      `N = input_coeffs.size()` determines the filter order.
   * @param output_coeffs Feedback (denominator) coefficients `a_0, a_1, …, a_{M-1}`.
   *                      Pass an empty vector for a pure FIR filter.
   *
   * @note Ring buffers are zero-initialised at construction; no further heap
   *       allocation occurs after this point.
   */
  DiscreteFilter(std::vector<G> input_coeffs, std::vector<G> output_coeffs)
  : input_coeffs_(input_coeffs),
    output_coeffs_(output_coeffs),
    inputs_num_(input_coeffs.size()),
    outputs_num_(output_coeffs.size()),
    inputs_(inputs_num_, static_cast<T>(0.f)),
    outputs_(outputs_num_, static_cast<T>(0.f))
  {}

  /**
   * @brief Processes one input sample and returns the filtered output.
   *
   * @details Pushes `new_input` into the input ring, computes the FIR sum,
   * then subtracts the IIR feedback sum, stores the output, and returns it.
   * No heap allocation occurs during this call.
   *
   * @param new_input New sample value of type `T`.
   * @returns Filtered output @f$ y[n] @f$.
   */
  virtual T update(T new_input)
  {
    inputs_.push(new_input);

    T sum_ = static_cast<T>(0.f);
    for (size_t i = 0; i < inputs_num_; ++i) {
      sum_ += (input_coeffs_[i] * inputs_[i]);
    }

    if (outputs_num_ == 0) {
      return sum_;
    }

    for (size_t j = 0; j < outputs_num_; ++j) {
      sum_ -= (output_coeffs_[j] * outputs_[j]);
    }

    outputs_.push(sum_);

    return sum_;
  }

  /**
   * @brief Resets the filter state to specified fill values.
   *
   * @param input_fill  Value to fill the input ring. Default 0.
   * @param output_fill Value to fill the output ring. Default 0.
   *
   * @note Call this at the start of a control session to prevent transients from
   *       stale state accumulated during calibration or idle time. No heap
   *       allocation occurs.
   */
  virtual void reset(T input_fill = static_cast<T>(0.f), T output_fill = static_cast<T>(0.f))
  {
    inputs_.fill_all(static_cast<T>(input_fill));
    outputs_.fill_all(static_cast<T>(output_fill));
  }

  /**
   * @brief Overwrites the most-recent output state (integrator back-calculation).
   *
   * @details Used by anti-windup logic to clamp the PI integrator state after
   * output saturation. Sets `y[n]` (the value stored in the output ring at age 0)
   * to `val`, so the next `update()` uses the clamped value as its `y[n-1]`.
   *
   * @param val Replacement output state value.
   */
  void set_output_state(T val)
  {
    if (!outputs_.empty()) outputs_[0] = val;
  }

private:
  std::vector<G>  input_coeffs_;  ///< Feedforward coefficients b_k
  std::vector<G>  output_coeffs_; ///< Feedback coefficients a_j

  size_t          inputs_num_;    ///< Order N of the FIR part
  size_t          outputs_num_;   ///< Order M of the IIR part
  RingBuffer<T>   inputs_;        ///< Input shift register x[n], x[n-1], …
  RingBuffer<T>   outputs_;       ///< Output shift register y[n-1], y[n-2], …
};


// ===========================================================================
// Butterworth2nd
// ===========================================================================

/**
 * @brief 2nd-order Butterworth low-pass filter discretised via bilinear transform.
 *
 * @details Uses frequency pre-warping to place the −3 dB cutoff exactly at
 * `cutoff_hz` in the discrete-time frequency domain:
 * @f[ \alpha = \frac{1}{\tan(\pi f_c / f_s)} \f]
 * @f[ \beta  = \alpha^2 + \sqrt{2}\,\alpha + 1 \f]
 *
 * Numerator (b) coefficients (amplitude-normalised):
 * @f[ b_0 = b_2 = \frac{1}{\beta}, \quad b_1 = \frac{2}{\beta} @f]
 *
 * Denominator (a) feedback coefficients:
 * @f[ a_0 = \frac{-2\alpha^2 + 2}{\beta}, \quad
 *     a_1 = \frac{\alpha^2 - \sqrt{2}\,\alpha + 1}{\beta} @f]
 *
 * @tparam T Signal type (typically `float`).
 *
 * @note The `α` pre-warp factor is the reciprocal of the Butterworth design
 * parameter commonly denoted `Ω_c` in textbooks; `α = cot(π f_c / f_s)`.
 */
template<typename T>
class Butterworth2nd : public DiscreteFilter<T, float>
{
public:
  Butterworth2nd() = default;
  ~Butterworth2nd() = default;

  /**
   * @brief Constructs the filter at the specified cutoff and sampling frequencies.
   * @param cutoff_hz  Desired −3 dB cutoff frequency [Hz].
   * @param sampling_hz Sampling (control loop) frequency [Hz].
   */
  Butterworth2nd(float cutoff_hz, float sampling_hz)
  : DiscreteFilter<T, float>(input_coeffs(cutoff_hz, sampling_hz),
      output_coeffs(cutoff_hz, sampling_hz))
  {}

private:
  /// @brief Computes the two IIR feedback coefficients.
  std::vector<float> output_coeffs(float f_c, float f_s)
  {
    T alpha = 1.f / tanf(M_PI * f_c / f_s);
    T beta = alpha * alpha + _SQRT_2_ * alpha + 1;

    return {(-2.f * alpha * alpha + 2.f) / beta, (alpha * alpha - _SQRT_2_ * alpha + 1.f) / beta};

  }

  /// @brief Computes the three FIR feedforward coefficients.
  std::vector<float> input_coeffs(float f_c, float f_s)
  {
    T alpha = 1.f / tanf(M_PI * f_c / f_s);
    T beta = alpha * alpha + _SQRT_2_ * alpha + 1;

    return {1.f / beta, 2.f / beta, 1.f / beta};

  }

};


// ===========================================================================
// PIController
// ===========================================================================

/**
 * @brief Discrete-time PI controller using the Tustin (trapezoidal) method.
 *
 * @details The continuous-time PI transfer function @f$ C(s) = K_p + K_i/s @f$
 * is discretised using the bilinear substitution @f$ s = \tfrac{2}{T}\tfrac{z-1}{z+1} @f$,
 * yielding the difference equation:
 * @f[
 *   u[n] = \underbrace{(K_p + \tfrac{K_i T}{2})}_{b_0} e[n]
 *         + \underbrace{(-K_p + \tfrac{K_i T}{2})}_{b_1} e[n-1]
 *         + u[n-1]
 * @f]
 * where @f$ e @f$ is the error input and @f$ u @f$ is the control output.
 * The feedback coefficient @f$ a_0 = -1 @f$ implements the accumulator (integrator).
 *
 * @tparam T Signal type (e.g., `float`, `QuadDirectValues<float>`).
 *
 * @note Anti-windup is implemented in `BrushlessController::feedback()` via
 *       output clamping and integrator back-calculation through `set_output_state()`.
 */
template<typename T>
class PIController : public DiscreteFilter<T, float>
{
public:
  PIController() = default;
  ~PIController() = default;

  /**
   * @brief Constructs the Tustin-discretised PI controller.
   *
   * @param Kp              Proportional gain.
   * @param Ki              Integral gain.
   * @param control_period_s Control period @f$ T @f$ in seconds.
   */
  PIController(float Kp, float Ki, float control_period_s)
  : DiscreteFilter<T, float>({Kp + 0.5f * Ki * control_period_s,
        -Kp + 0.5f * Ki * control_period_s},
      {-1.f})
  {}
};


// ===========================================================================
// MotorFeedforward
// ===========================================================================

/**
 * @brief Bilinear (Tustin) inversion of the first-order RL motor model for feedforward.
 *
 * @details Models the per-phase motor winding as a series RL circuit:
 * @f[ V(s) = (R + sL)\, I(s) @f]
 * The bilinear-transform inversion yields the discrete-time voltage predictor:
 * @f[
 *   V[n] = \underbrace{\!\left(\frac{2L}{T}+R\right)\!}_{A}\! I[n]
 *         + \underbrace{\!\left(-\frac{2L}{T}+R\right)\!}_{B}\! I[n-1]
 *         - V[n-1]
 * @f]
 * This is used in `BrushlessController::feedforward()` (inline implementation)
 * to pre-compute the voltage required to achieve a desired current trajectory.
 *
 * @note This class is defined for documentation completeness. The controller uses
 *       an inline implementation of the same formula rather than instantiating
 *       `MotorFeedforward` as a filter object, because the phase-current and
 *       phase-voltage states are managed by the controller's own member variables.
 */
class MotorFeedforward : public DiscreteFilter<PhaseValues<float>, float>
{
public:
  MotorFeedforward() = default;
  ~MotorFeedforward() = default;

  /**
   * @brief Constructs the bilinear RL inversion filter.
   *
   * @param phase_R         Per-phase winding resistance [Ω].
   * @param phase_L         Per-phase winding inductance [H].
   * @param control_period_s Control period @f$ T @f$ [s].
   */
  MotorFeedforward(float phase_R, float phase_L, float control_period_s)
  : DiscreteFilter<PhaseValues<float>, float>({2.f * phase_L / control_period_s + phase_R,
        -2.f * phase_L / control_period_s + phase_R}, {1.f})
  {}

};


// ===========================================================================
// VelocityPLL
// ===========================================================================

/**
 * @brief 2nd-order phase-locked loop for simultaneous position and velocity estimation.
 *
 * @details Tracks encoder angle with a recursive state observer. The PLL drives
 * a position estimate toward the measured angle using a proportional-integral
 * loop, and its integral state is the velocity estimate:
 * @f[
 *   e[n]        = \theta_\text{meas}[n] - \hat\theta[n-1]
 * @f]
 * @f[
 *   \hat\theta[n] = \hat\theta[n-1] + (\hat\omega[n-1] + K_p\,e[n])\,T_s
 * @f]
 * @f[
 *   \hat\omega[n] = \hat\omega[n-1] + K_i\,e[n]\,T_s
 * @f]
 *
 * Gains are derived from the desired closed-loop natural frequency and damping:
 * @f[ K_p = 2\zeta\omega_n, \quad K_i = \omega_n^2 \f]
 *
 * For ζ = 1/√2 (Butterworth), the −3 dB velocity bandwidth equals `bandwidth_hz`.
 *
 * @note Uses Forward Euler integration. Stable for bandwidth_hz ≪ sampling_hz / (2π).
 *       At 10 kHz sampling, bandwidths up to ~500 Hz are well-behaved.
 */
class VelocityPLL
{
public:
  VelocityPLL() = default;
  ~VelocityPLL() = default;

  /**
   * @brief Constructs the PLL with specified bandwidth and damping.
   *
   * @param bandwidth_hz     Closed-loop −3 dB bandwidth [Hz].
   * @param damping          Damping ratio ζ (1/√2 ≈ 0.707 gives Butterworth response).
   * @param control_period_s Control loop period [s].
   */
  VelocityPLL(float bandwidth_hz, float damping, float control_period_s)
  {
    float omega_n = _2_PI_ * bandwidth_hz;
    kp_ = 2.f * damping * omega_n;
    ki_ = omega_n * omega_n;
    ts_ = control_period_s;
  }

  /**
   * @brief Advances the PLL by one sample.
   *
   * @param measured_pos Measured shaft angle [rad], multi-turn unwrapped.
   */
  void update(float measured_pos)
  {
    float error = measured_pos - pos_est_;
    pos_est_ += (vel_est_ + kp_ * error) * ts_;
    vel_est_ += ki_ * error * ts_;
  }

  /// @brief Returns the PLL position estimate [rad].
  float get_position() const { return pos_est_; }

  /// @brief Returns the PLL velocity estimate [rad/s].
  float get_velocity() const { return vel_est_; }

  /**
   * @brief Resets PLL state to suppress startup transients.
   *
   * @param position Initial position estimate [rad]. Default 0.
   * @param velocity Initial velocity estimate [rad/s]. Default 0.
   */
  void reset(float position = 0.f, float velocity = 0.f)
  {
    pos_est_ = position;
    vel_est_ = velocity;
  }

private:
  float kp_      = 0.f;    ///< Proportional gain [rad/s per rad error]
  float ki_      = 0.f;    ///< Integral gain [(rad/s)/s per rad error]
  float ts_      = 1e-4f;  ///< Sample period [s]
  float pos_est_ = 0.f;    ///< Estimated position [rad]
  float vel_est_ = 0.f;    ///< Estimated velocity [rad/s]
};

#endif // DISCRETE_FILTER_HPP
