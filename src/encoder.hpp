#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <imxrt.h>
#include <SPI.h>
#include <wiring.h>
#include "helpers.hpp"
#include "discrete_filter.hpp"

struct Angle
{
  int rotations = 0;
  float radians = 0.f;

  int direction = 1;

  float get_full_angle() const
  {
    return static_cast<float>(direction) * (static_cast<float>(rotations) * _2_PI_ + radians);
  }

  float get_angle() const
  {
    return static_cast<float>(direction) * radians;
  }

  void update_angle(float new_radians)
  {
    if(new_radians < 0){
      return;
    }
    auto delta_radians = new_radians - radians;
    if (abs(delta_radians) > (PI)) {
      rotations += (delta_radians > 0) ? -1 : 1;
    }
    radians = new_radians;
  }

  void reset() {
    radians = 0.f;
    rotations = 0;
  }

};


class AbsoluteEncoder
{
public:
  AbsoluteEncoder() = default;
  ~AbsoluteEncoder() = default;

  /// @brief read the encoder
  /// @returns the angle is radians (0, 2PI)
  virtual float read(); 

};
#endif
