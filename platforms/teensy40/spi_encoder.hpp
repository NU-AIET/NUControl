#ifndef SPI_ENCODER_HPP
#define SPI_ENCODER_HPP

#include "../../core/encoder_interface.hpp"
#include <Arduino.h>
#include <SPI.h>

class SPIEncoder : public IAbsoluteEncoder
{
public:
  SPIEncoder() = default;
  ~SPIEncoder() = default;
  SPIEncoder(
    const uint16_t read_cmd,
    SPIClass & spi,
    const int chip_select,
    const int speed = 10000000,
    const int bit_order = MSBFIRST,
    const int mode = SPI_MODE1,
    const int res = 14
  )
  : read_cmd_(read_cmd),
    SPI_(spi),
    settings_(SPISettings(speed, bit_order, mode)),
    cs_(chip_select),
    max_read_((1 << res))
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
    if(glitch_filter_enable && (read == 0 || read == 16383))
    {
      return -1.f;
    }
    return static_cast<float>(read) / static_cast<float>(max_read_ - 1) * _2_PI_;
  }

  void set_glitch_filter_state(bool state)
  {
    glitch_filter_enable = state;

  }

  uint16_t read_raw()
  {
    digitalWriteFast(cs_, LOW);
    SPI_.transfer16(read_cmd_);
    digitalWriteFast(cs_, HIGH);
    delayNanoseconds(400);
    digitalWriteFast(cs_, LOW);
    uint16_t raw = SPI_.transfer16(0x000);
    digitalWriteFast(cs_, HIGH);
    return ((raw >> 0) << 0) & (max_read_ - 1);
  }

private:
  const uint16_t read_cmd_;
  SPIClass & SPI_;
  const SPISettings settings_;
  const int cs_;
  const int max_read_;
  bool glitch_filter_enable = true;
};
#endif
