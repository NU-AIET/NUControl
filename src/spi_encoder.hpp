#ifndef SPI_ENCODER_HPP
#define SPI_ENCODER_HPP

#include <Arduino.h>
#include <imxrt.h>
#include <SPI.h>
#include <wiring.h>
#include "helpers.hpp"
#include "discrete_filter.hpp"

// enum AngleUnit{
//   RADS,
//   REVS,
//   DEGREES,
// };

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

};


class SPIEncoder
{
public:
  SPIEncoder() = default;
  ~SPIEncoder() = default;
  SPIEncoder(
    SPIClass & spi,
    const int chip_select,
    const int speed = 10000000,
    const int bit_order = MSBFIRST,
    const int mode = SPI_MODE1
  )
  : SPI_(spi),
    settings_(SPISettings(speed, bit_order, mode)),
    cs_(chip_select)
  {
    pinMode(cs_, OUTPUT);
    digitalWriteFast(cs_, HIGH);
    SPI_.begin();
    SPI_.beginTransaction(settings_);
  }


  /// @brief read the encoder
  /// @returns the angle is radians (0, 2PI)
  float read()
  {
    auto read = read_raw();
    if(read == 0 || read == 16383)
    {
      return -1.f;
    }
    return static_cast<float>(read_raw()) / 16383.0f * _2_PI_;
  }

  uint16_t read_raw()
  {
    digitalWriteFast(cs_, LOW);
    SPI_.transfer16(u_read_cmd);
    digitalWriteFast(cs_, HIGH);
    delayNanoseconds(400);
    digitalWriteFast(cs_, LOW);
    uint16_t raw = SPI_.transfer16(0x000);
    digitalWriteFast(cs_, HIGH);
    return ((raw >> 0) << 0) & 16383;

  }

private:
  SPIClass & SPI_;
  const SPISettings settings_;
  const int cs_;
  const uint16_t read_cmd_ = (0b11 << 14) | 0x3FFF;
  const uint16_t u_read_cmd = (0b10 << 14) | 0x3FFE;

};
#endif
