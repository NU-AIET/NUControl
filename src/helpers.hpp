/**
 * @file helpers.hpp
 * @brief Math utility constants and functions for FOC computations.
 *
 * @details Provides compile-time constants for commonly used irrational numbers
 * in Clarke/Park transform arithmetic, a precision-safe angle normaliser, and a
 * paired sincos function that normalises its input before calling the
 * floating-point trig library.
 */

#ifndef HELPERS_HPP
#define HELPERS_HPP
#include <math.h>
#include <utility>

// ---------------------------------------------------------------------------
// Compile-time constants
// ---------------------------------------------------------------------------

/** @brief \f$ 1/\sqrt{3} \approx 0.5774 \f$ — used in amplitude-invariant Clarke transform. */
constexpr float _1__SQRT_3_ = 0.57735026919f;

/** @brief \f$ 2/\sqrt{3} \approx 1.1547 \f$ — used in amplitude-invariant Clarke transform. */
constexpr float _2__SQRT_3_ = 1.15470053838f;

/** @brief \f$ \sqrt{3}/2 \approx 0.8660 \f$ — used in inverse Clarke transform. */
constexpr float _SQRT_3__2_ = 0.866025403784f;

/** @brief \f$ 2\pi \approx 6.2832 \f$ — full rotation in radians. */
constexpr float _2_PI_ = 2.0f * M_PI;

/** @brief \f$ \sqrt{2} \approx 1.4142 \f$ — used in Butterworth filter coefficient computation. */
constexpr float _SQRT_2_ = 1.41421356237f;

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/**
 * @brief Tests whether two values are within a tolerance of each other.
 *
 * @tparam T Numeric type supporting `abs()` and subtraction.
 * @param a   First value.
 * @param b   Second value.
 * @param tol Tolerance (inclusive). Default 0.001.
 * @returns `true` if `|a − b| < tol`, `false` otherwise.
 */
template<typename T>
bool near_zero(T a, T b, float tol = 0.001)
{
  return abs(a - b) < static_cast<T>(tol);
}

/**
 * @brief Wraps an angle in radians to the half-open interval (−π, π].
 *
 * @details Uses the identity:
 * @f[ \theta_{\text{norm}} = \text{fmod}(\theta + \pi,\; 2\pi) - \pi @f]
 *
 * This is equivalent to iteratively adding or subtracting 2π until the result
 * is in (−π, π], but executes in O(1) time using `fmod`.
 *
 * @tparam T Floating-point type.
 * @param radians Input angle in radians (any range).
 * @returns Equivalent angle in (−π, π].
 *
 * @note Values exactly equal to −π may map to either −π or +π depending on
 * floating-point rounding; for FOC applications this boundary is not critical.
 */
template<typename T>
T normalize_angle(T radians)
{
  return fmod(radians + M_PI, _2_PI_) - M_PI;
}

/**
 * @brief Computes sine and cosine together in a single call.
 *
 * @details Normalises the input to (−π, π] before calling `sinf` / `cosf`,
 * which improves floating-point accuracy for inputs far from zero (large
 * multiturn accumulated angles).
 *
 * @param radians Input angle in radians.
 * @returns `std::pair<float, float>{sin(radians), cos(radians)}`.
 *
 * @note The normalisation adds one `fmod` call but is negligible compared to
 * the trig functions themselves. On Cortex-M7, `sinf` / `cosf` use the
 * hardware FPU and are approximately 20 cycles each.
 *
 * @todo Consider replacing with `arm_sin_cos_f32()` from CMSIS-DSP (uses a
 *       256-entry lookup table + linear interpolation, faster than FPU sinf/cosf).
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpsabi"
inline std::pair<float, float> nu_sincos(float radians)
{
  // Normalise to (−π, π] for best floating-point accuracy
  auto rads = normalize_angle(radians);

  std::pair<float, float> sincos;

  sincos.first = sinf(rads);
  sincos.second = cosf(rads);

  return sincos;
}
#pragma GCC diagnostic pop


#endif // HELPERS_HPP
