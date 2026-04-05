/**
 * @file transformations.hpp
 * @brief Coordinate-frame structs and Clarke/Park transform functions for FOC.
 *
 * @details Defines the three coordinate frames used in field-oriented control:
 *
 * | Frame        | Type                 | Axes        |
 * |-------------|----------------------|-------------|
 * | Phase (abc) | `PhaseValues<T>`     | a, b, c     |
 * | Stationary  | `AlphaBetaValues<T>` | α, β        |
 * | Rotating    | `QuadDirectValues<T>`| q (torque), d (flux) |
 *
 * ## Clarke Transform (abc → αβ)
 * Amplitude-invariant form (assumes `I_a + I_b + I_c = 0`, so `I_c` is not needed):
 * @f[
 *   \alpha = I_a, \qquad
 *   \beta  = \frac{1}{\sqrt{3}}\,I_a + \frac{2}{\sqrt{3}}\,I_b
 * @f]
 *
 * ## Park Transform (αβ → dq)
 * Sign convention in this implementation:
 * @f[
 *   q = -\sin\theta\,\alpha + \cos\theta\,\beta, \qquad
 *   d =  \cos\theta\,\alpha + \sin\theta\,\beta
 * @f]
 * where @f$ \theta @f$ is the electrical angle in radians.
 *
 * @note Both the forward and inverse transforms are implemented as free
 *       template functions so they work for any numeric type `T` that
 *       supports the required arithmetic operators.
 */

#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP
#include "helpers.hpp"

// ===========================================================================
// PhaseValues
// ===========================================================================

/**
 * @brief Three-phase (abc) coordinate frame values.
 *
 * @details Holds per-phase quantities such as currents [A] or voltages [V]
 * in the stationary abc frame. The scalar constructor initialises all three
 * phases to the same value, enabling use as the fill type for
 * `DiscreteFilter<PhaseValues<T>, float>`.
 *
 * @tparam T Element type (typically `float`).
 */
template<typename T>
struct PhaseValues
{
  T a; ///< Phase A value (e.g., current in A or voltage in V).
  T b; ///< Phase B value.
  T c; ///< Phase C value.

  PhaseValues() = default;
  ~PhaseValues() = default;

  /**
   * @brief Constructs with independent per-phase values.
   * @param a_ Phase A.
   * @param b_ Phase B.
   * @param c_ Phase C.
   */
  PhaseValues(T a_, T b_, T c_)
  : a(a_),
    b(b_),
    c(c_)
  {}

  /**
   * @brief Scalar broadcast constructor — sets all phases to `val`.
   *
   * @details Required for compatibility with `DiscreteFilter<PhaseValues<T>, G>`,
   * which zero-initialises its shift register via `static_cast<T>(0.f)`.
   *
   * @param val Value to assign to all three phases.
   */
  PhaseValues(T val)
  : a(val),
    b(val),
    c(val)
  {}

  /// @brief Component-wise addition assignment.
  PhaseValues<T> & operator+=(const PhaseValues<T> & other)
  {
    a += other.a;
    b += other.b;
    c += other.c;
    return *this;
  }

  /// @brief Component-wise subtraction assignment.
  PhaseValues<T> & operator-=(const PhaseValues<T> & other)
  {
    a -= other.a;
    b -= other.b;
    c -= other.c;
    return *this;
  }
};


// ===========================================================================
// AlphaBetaValues
// ===========================================================================

/**
 * @brief Stationary two-phase (αβ) coordinate frame values.
 *
 * @details Intermediate frame produced by the Clarke transform and consumed
 * by the Park transform. The α axis is aligned with the phase-A axis;
 * the β axis leads α by 90°.
 *
 * @tparam T Element type (typically `float`).
 */
template<typename T>
struct AlphaBetaValues
{
  T alpha; ///< α-axis component — aligned with phase A.
  T beta;  ///< β-axis component — 90° ahead of α.
};


// ===========================================================================
// QuadDirectValues
// ===========================================================================

/**
 * @brief Rotating dq (quadrature–direct) coordinate frame values.
 *
 * @details The dq frame rotates synchronously with the rotor flux vector at
 * electrical angle @f$ \theta_e @f$. In steady state, all quantities become
 * DC, enabling simple PI current control:
 * - **q** (quadrature): torque-producing component @f$ I_q @f$; controls @f$ \tau = k_T I_q @f$.
 * - **d** (direct): flux-producing component @f$ I_d @f$; held at zero for
 *   maximum torque per ampere (MTPA) with surface-mounted magnets.
 *
 * The scalar constructor enables use as a `DiscreteFilter` state type.
 *
 * @tparam T Element type (typically `float`).
 */
template<typename T>
struct QuadDirectValues
{
  T q; ///< Quadrature (torque) component.
  T d; ///< Direct (flux) component.

  QuadDirectValues() = default;
  ~QuadDirectValues() = default;

  /**
   * @brief Constructs with independent q and d values.
   * @param q_ Quadrature component.
   * @param d_ Direct component.
   */
  QuadDirectValues(T q_, T d_)
  : q(q_),
    d(d_)
  {}

  /**
   * @brief Scalar broadcast constructor — sets both components to `val`.
   *
   * @details Required for compatibility with `DiscreteFilter<QuadDirectValues<T>, G>`.
   * @param val Value to assign to q and d.
   */
  QuadDirectValues(T val)
  : q(val),
    d(val)
  {}

  /// @brief Component-wise addition assignment.
  QuadDirectValues<T> & operator+=(const QuadDirectValues<T> & other)
  {
    q += other.q;
    d += other.d;
    return *this;
  }

  /// @brief Component-wise subtraction assignment.
  QuadDirectValues<T> & operator-=(const QuadDirectValues<T> & other)
  {
    q -= other.q;
    d -= other.d;
    return *this;
  }
};


// ===========================================================================
// PhaseValues free operators
// ===========================================================================

/// @brief Scales a `PhaseValues` by scalar `s` (phs * s).
template<typename T>
PhaseValues<T> operator*(PhaseValues<T> phs, T s)
{
  return {s * phs.a, s * phs.b, s * phs.c};
}

/// @brief Scales a `PhaseValues` by scalar `s` (s * phs).
template<typename T>
PhaseValues<T> operator*(T s, PhaseValues<T> phs)
{
  return {s * phs.a, s * phs.b, s * phs.c};
}

/// @brief Component-wise addition of two `PhaseValues`.
template<typename T>
PhaseValues<T> operator+(PhaseValues<T> phs_a, PhaseValues<T> phs_b)
{
  return {phs_a.a + phs_b.a, phs_a.b + phs_b.b, phs_a.c + phs_b.c};
}

/// @brief Component-wise subtraction of two `PhaseValues`.
template<typename T>
PhaseValues<T> operator-(PhaseValues<T> phs_a, PhaseValues<T> phs_b)
{
  return {phs_a.a - phs_b.a, phs_a.b - phs_b.b, phs_a.c - phs_b.c};
}


// ===========================================================================
// QuadDirectValues free operators
// ===========================================================================

/// @brief Scales a `QuadDirectValues` by scalar `s` (qd * s).
template<typename T>
QuadDirectValues<T> operator*(QuadDirectValues<T> qd, T s)
{
  return {s * qd.q, s * qd.d};
}

/// @brief Scales a `QuadDirectValues` by scalar `s` (s * qd).
template<typename T>
QuadDirectValues<T> operator*(T s, QuadDirectValues<T> qd)
{
  return {s * qd.q, s * qd.d};
}

/// @brief Component-wise addition of two `QuadDirectValues`.
template<typename T>
QuadDirectValues<T> operator+(QuadDirectValues<T> qd_a, QuadDirectValues<T> qd_b)
{
  return {qd_a.q + qd_b.q, qd_a.d + qd_b.d};
}

/// @brief Component-wise subtraction of two `QuadDirectValues`.
template<typename T>
QuadDirectValues<T> operator-(QuadDirectValues<T> qd_a, QuadDirectValues<T> qd_b)
{
  return {qd_a.q - qd_b.q, qd_a.d - qd_b.d};
}


// ===========================================================================
// Clarke transform
// ===========================================================================

/**
 * @brief Forward Clarke transform: abc → αβ (amplitude-invariant).
 *
 * @details Projects the three-phase abc frame onto the orthogonal αβ frame.
 * Uses the KCL constraint @f$ I_a + I_b + I_c = 0 @f$ so that only two
 * measurements are required:
 * @f[
 *   \alpha = I_a, \qquad
 *   \beta  = \frac{1}{\sqrt{3}}\,I_a + \frac{2}{\sqrt{3}}\,I_b
 * @f]
 *
 * Constants `_1__SQRT_3_` and `_2__SQRT_3_` are defined in `helpers.hpp`.
 *
 * @tparam T Numeric type (must support `+` and scalar `*`).
 * @param phases  Three-phase values @f$ (I_a, I_b, I_c) @f$.
 * @returns       Stationary frame values @f$ (\alpha, \beta) @f$.
 */
template<typename T>
AlphaBetaValues<T> phases_to_alphabeta(PhaseValues<T> phases)
{
  AlphaBetaValues<T> alphabeta;
  alphabeta.alpha = phases.a;
  alphabeta.beta = _1__SQRT_3_ * phases.a + _2__SQRT_3_ * phases.b;
  return alphabeta;
}

/**
 * @brief Inverse Clarke transform: αβ → abc.
 *
 * @details Reconstructs the three-phase frame from the stationary αβ frame:
 * @f[
 *   I_a =  \alpha, \qquad
 *   I_b = -\tfrac{1}{2}\alpha + \tfrac{\sqrt{3}}{2}\beta, \qquad
 *   I_c = -\tfrac{1}{2}\alpha - \tfrac{\sqrt{3}}{2}\beta
 * @f]
 *
 * Constants `_SQRT_3__2_` is defined in `helpers.hpp`.
 *
 * @tparam T Numeric type.
 * @param alphabeta  Stationary frame values @f$ (\alpha, \beta) @f$.
 * @returns          Three-phase values @f$ (I_a, I_b, I_c) @f$.
 */
template<typename T>
PhaseValues<T> alphabeta_to_phases(AlphaBetaValues<T> alphabeta)
{
  PhaseValues<T> phases;
  phases.a = alphabeta.alpha;
  phases.b = -0.5 * alphabeta.alpha + _SQRT_3__2_ * alphabeta.beta;
  phases.c = -0.5 * alphabeta.alpha - _SQRT_3__2_ * alphabeta.beta;
  return phases;
}


// ===========================================================================
// Park transform
// ===========================================================================

/**
 * @brief Forward Park transform: αβ → dq.
 *
 * @details Rotates the stationary αβ frame into the rotor-synchronous dq frame
 * using electrical angle @f$ \theta @f$:
 * @f[
 *   q = -\sin\theta\,\alpha + \cos\theta\,\beta, \qquad
 *   d =  \cos\theta\,\alpha + \sin\theta\,\beta
 * @f]
 *
 * @note The q-axis leads the d-axis by 90° in this convention, which places
 *       the q-axis in the direction of maximum torque production for a rotor
 *       with d-axis aligned to the flux axis.
 *
 * @tparam T Numeric type.
 * @param alphabeta   Stationary frame values @f$ (\alpha, \beta) @f$.
 * @param eangle_rads Electrical angle @f$ \theta @f$ [rad].
 * @returns           Rotating frame values @f$ (q, d) @f$.
 */
template<typename T>
QuadDirectValues<T> alphabeta_to_quaddirect(AlphaBetaValues<T> alphabeta, float eangle_rads)
{
  auto sc = nu_sincos(eangle_rads);
  return {-sc.first * alphabeta.alpha + sc.second * alphabeta.beta,
    sc.second * alphabeta.alpha + sc.first * alphabeta.beta};
}

/**
 * @brief Inverse Park transform: dq → αβ.
 *
 * @details Rotates the dq frame back to the stationary αβ frame:
 * @f[
 *   \alpha = \cos\theta\,d - \sin\theta\,q, \qquad
 *   \beta  = \sin\theta\,d + \cos\theta\,q
 * @f]
 *
 * @tparam T Numeric type.
 * @param quaddirect  Rotating frame values @f$ (q, d) @f$.
 * @param eangle_rads Electrical angle @f$ \theta @f$ [rad].
 * @returns           Stationary frame values @f$ (\alpha, \beta) @f$.
 */
template<typename T>
AlphaBetaValues<T> quaddirect_to_alphabeta(QuadDirectValues<T> quaddirect, float eangle_rads)
{
  auto sc = nu_sincos(eangle_rads);
  return {sc.second * quaddirect.d - sc.first * quaddirect.q,
    sc.first * quaddirect.d + sc.second * quaddirect.q};
}


// ===========================================================================
// Combined transforms
// ===========================================================================

/**
 * @brief Combined inverse Park + inverse Clarke: dq → abc.
 *
 * @details Convenience wrapper that calls `quaddirect_to_alphabeta()` then
 * `alphabeta_to_phases()`. Used in `BrushlessController::update_control()`
 * to convert voltage commands from the dq frame back to three-phase duties.
 *
 * @tparam T Numeric type.
 * @param quaddirect  Rotating frame values @f$ (q, d) @f$.
 * @param eangle_rads Electrical angle @f$ \theta @f$ [rad].
 * @returns           Three-phase values @f$ (V_a, V_b, V_c) @f$.
 */
template<typename T>
PhaseValues<T> quaddirect_to_phases(QuadDirectValues<T> quaddirect, float eangle_rads)
{
  return alphabeta_to_phases(quaddirect_to_alphabeta(quaddirect, eangle_rads));
}

/**
 * @brief Combined Clarke + Park: abc → dq.
 *
 * @details Convenience wrapper that calls `phases_to_alphabeta()` then
 * `alphabeta_to_quaddirect()`. Used in `BrushlessController::update_sensors()`
 * to project measured phase currents directly into the dq frame.
 *
 * @tparam T Numeric type.
 * @param phases      Three-phase values @f$ (I_a, I_b, I_c) @f$.
 * @param eangle_rads Electrical angle @f$ \theta @f$ [rad].
 * @returns           Rotating frame values @f$ (I_q, I_d) @f$.
 */
template<typename T>
QuadDirectValues<T> phases_to_quaddirect(PhaseValues<T> phases, float eangle_rads)
{
  return alphabeta_to_quaddirect(phases_to_alphabeta(phases), eangle_rads);
}


#endif // TRANSFORMATION_HPP
