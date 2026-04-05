/**
 * @file spi_encoder.hpp
 * @brief SPI absolute encoder driver for 14-bit (and configurable) magnetic encoders.
 *
 * @details Implements `AbsoluteEncoder::read()` using a two-transaction SPI protocol:
 *
 * **Transaction 1 — Command frame** (sends read command, return ignored):
 * ```
 * CS ↓ → transfer16(read_cmd_) → CS ↑ → delay 400 ns
 * ```
 * **Transaction 2 — Data frame** (sends NOP 0x0000, captures previous conversion):
 * ```
 * CS ↓ → transfer16(0x0000) → CS ↑
 * ```
 * The 400 ns inter-frame gap satisfies the AS5147/AS5048 tCSn hold requirement.
 * The data frame returns the angle result latched by the encoder at the end of
 * transaction 1's falling CS edge.
 *
 * **Glitch filter**: rejects raw readings of 0 and 16383 (0x3FFF), which are the
 * two saturation codes produced by the AS5047/AS5147 when magnetic field strength
 * is out of range. These return −1.0 from `read()`, which `Angle::update_angle()`
 * silently discards.
 *
 * @note The SPI transaction is begun in the constructor (`SPI_.beginTransaction()`)
 *       and is never ended — `SPIEncoder` assumes exclusive ownership of the bus
 *       for the lifetime of the object. If multiple SPI devices are present,
 *       move `beginTransaction` / `endTransaction` into `read_raw()`.
 *
 * @todo Replace the blocking SPI transfers with DMA-driven LPSPI to eliminate
 *       the last source of blocking I/O from the control ISR and allow the
 *       encoder read to be pre-fetched in parallel with the ADC conversion.
 *       (See deferred improvement R2.)
 */

#ifndef SPI_ENCODER_HPP
#define SPI_ENCODER_HPP

#include "encoder.hpp"

/**
 * @brief SPI-based absolute encoder driver.
 *
 * @details Supports any SPI encoder that responds to a 16-bit read command and
 * returns a 16-bit angle word on the next frame (e.g., AMS AS5047, AS5147, AS5048).
 * Resolution is configurable via the `res` constructor parameter; the default 14 bits
 * maps to the AS5047/AS5147 full-scale count of 16384.
 *
 * Inherits `AbsoluteEncoder` and overrides `read()`.
 */
class SPIEncoder : public AbsoluteEncoder
{
public:
  SPIEncoder() = default;
  ~SPIEncoder() = default;

  /**
   * @brief Constructs and initialises the SPI encoder.
   *
   * @details Configures the chip-select pin, calls `SPI_.begin()`, and opens
   * a persistent SPI transaction at the specified settings.
   *
   * @param read_cmd   16-bit command word sent on transaction 1 (e.g., 0x3FFF for AS5047 angle register).
   * @param spi        Reference to the `SPIClass` instance (e.g., `SPI`).
   * @param chip_select Arduino pin number used as chip-select (active-low).
   * @param speed      SPI clock rate [Hz]. Default: 10 MHz (within AS5047 spec at 3.3 V).
   * @param bit_order  `MSBFIRST` or `LSBFIRST`. Default: `MSBFIRST`.
   * @param mode       SPI mode. Default: `SPI_MODE1` (CPOL=0, CPHA=1 per AS5047 datasheet).
   * @param res        Encoder resolution in bits. Default: 14 (→ 16384 counts/rev).
   */
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

  /**
   * @brief Reads the current shaft angle via the two-frame SPI protocol.
   *
   * @details Calls `read_raw()` and converts to radians. If the glitch filter
   * is enabled and the raw reading equals 0 or 16383 (AS5047 magnetic field
   * error codes), returns −1.0 as a sentinel; the caller (`Angle::update_angle()`)
   * discards negative values without updating state.
   *
   * @returns Shaft angle [rad] in [0, 2π), or −1.0 on a glitch-filtered frame.
   *
   * @note Blocking — takes approximately 3–4 µs at 10 MHz SPI clock due to the
   *       two 16-bit transfers and the 400 ns inter-frame delay.
   *
   * @todo Migrate to DMA-driven LPSPI to eliminate blocking time from the ISR.
   */
  float read()
  {
    auto read = read_raw();
    if(glitch_filter_enable && (read == 0 || read == 16383))
    {
      return -1.f;
    }
    return static_cast<float>(read) / static_cast<float>(max_read_ - 1) * _2_PI_;
  }

  /**
   * @brief Enables or disables the glitch filter.
   *
   * @details When enabled (default), raw readings of 0 and 0x3FFF are treated as
   * magnetic field errors and discarded. Disable only for debugging or if the
   * encoder operates near the extreme field conditions that produce these codes
   * legitimately.
   *
   * @param state `true` to enable glitch filtering, `false` to disable.
   */
  void set_glitch_filter_state(bool state)
  {
    glitch_filter_enable = state;
  }

  /**
   * @brief Executes the two-frame SPI protocol and returns the raw 14-bit count.
   *
   * @details
   * - Frame 1: CS ↓, `transfer16(read_cmd_)`, CS ↑, 400 ns delay.
   * - Frame 2: CS ↓, `transfer16(0x0000)`, CS ↑.
   *
   * The result is masked to `(max_read_ − 1)` to strip status/parity bits
   * that appear in the upper bits of some encoder responses.
   *
   * @returns Raw position count in [0, max_read_ − 1].
   */
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
  const uint16_t  read_cmd_;           ///< Command word for transaction 1.
  SPIClass &      SPI_;                ///< Reference to the SPI bus.
  const SPISettings settings_;         ///< Clock, bit order, and mode.
  const int       cs_;                 ///< Chip-select pin (active-low).
  const int       max_read_;           ///< Full-scale count = 2^res.
  bool            glitch_filter_enable = true; ///< Reject 0 and 16383 when true.
};

#endif // SPI_ENCODER_HPP
