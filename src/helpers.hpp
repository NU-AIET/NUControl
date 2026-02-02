#ifndef HELPERS_HPP
#define HELPERS_HPP
#include <math.h>
#include <utility>

constexpr float _1__SQRT_3_ = 0.57735026919f;
constexpr float _2__SQRT_3_ = 1.15470053838f;
constexpr float _SQRT_3__2_ = 0.866025403784f;
constexpr float _2_PI_ = 2.0f * M_PI;
constexpr float _SQRT_2_ = 1.41421356237f;


/// @brief Compare to precision values
/// @tparam T type of values to check
/// @param a - first value to check
/// @param b - second value to check
/// @param tol - tolerance to check within
/// @returns true if the difference is less than tolerance, flase otherwise
template<typename T>
bool near_zero(T a, T b, float tol = 0.001)
{
  return abs(a - b) < static_cast<T>(tol);
}

/// @brief Brings an angle into a standardized range where functions can operate efficently upon
/// @tparam T type of angle unit
/// @param radians - the angle in radians
/// @returns the radians in the range (-PI, PI)
template<typename T>
T normalize_angle(T radians)
{
  while (radians > M_PI) {
    radians -= _2_PI_;
  }
  while (radians < -M_PI) {
    radians += _2_PI_;
  }
  return radians;
}

/// @brief Gets sine and cosine in a single function
/// @param radians - the angle in radians
/// @returns sin, cos of the given angle
/// \note used here so custom implementation is easier
std::pair<float, float> nu_sincos(float radians)
{
  // Now zoned between -PI and PI
  auto rads = normalize_angle(radians);

  std::pair<float, float> sincos;

  sincos.first = sinf(rads);
  sincos.second = cosf(rads);

  return sincos;
}


#endif
