#ifndef ENCODER_INTERFACE_HPP
#define ENCODER_INTERFACE_HPP
#include "helpers.hpp"

// Tracks cumulative shaft angle across multiple rotations.
struct Angle
{
  int rotations = 0;
  float radians = 0.f;
  int direction = 1;

  float get_full_angle() const
  {
    return static_cast<float>(direction) * (static_cast<float>(rotations) * _2_PI_ + radians);
  }

  float get_angle() const { return static_cast<float>(direction) * radians; }

  void update_angle(float new_radians)
  {
    if (new_radians < 0) { return; }
    auto delta = new_radians - radians;
    if (delta > _2_PI_ / 2.f) { rotations -= 1; }
    else if (delta < -_2_PI_ / 2.f) { rotations += 1; }
    radians = new_radians;
  }

  void reset() { radians = 0.f; rotations = 0; }
};

class IAbsoluteEncoder
{
public:
  virtual ~IAbsoluteEncoder() = default;
  // Returns shaft angle in radians, in the range [0, 2*pi).
  virtual float read() = 0;
};

#endif
