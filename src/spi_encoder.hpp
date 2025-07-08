#ifndef SPI_ENCODER_HPP
#define SPI_ENCODER_HPP

#include <Arduino.h>
#include <imxrt.h>
#include <SPI.h>
#include <wiring.h>
#include "helpers.hpp"

struct Angle
{
  int rotations = 0;
  float radians = 0.f;

  float get_full_angle() const
  {
    return static_cast<float>(rotations) * _2_PI_ + radians;
  }

  float get_angle() const
  {
    return radians;
  }

  void update_angle(float new_radians)
  {
    auto delta_radians = new_radians - radians;
    // Serial.println(1000.f*delta_radians);
    if (abs(delta_radians) > (0.5 * _2_PI_)) {
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

  float read()
  {
    digitalWriteFast(cs_, LOW);
    SPI_.transfer16(read_cmd_);
    digitalWriteFast(cs_, HIGH);
    delayNanoseconds(400);
    digitalWriteFast(cs_, LOW);
    uint16_t raw = SPI_.transfer16(read_cmd_);
    digitalWriteFast(cs_, HIGH);
    return (raw & 16383) / 16383.0f * _2_PI_;
  }

private:
  SPIClass & SPI_;
  const SPISettings settings_;
  const int cs_;
  const uint16_t read_cmd_ = (0b11 << 14) | 0x3FFF;

};
#endif
