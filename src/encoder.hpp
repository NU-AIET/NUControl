/**
 * @file encoder.hpp
 * @brief Multi-turn angle tracking struct and abstract absolute encoder interface.
 *
 * @details Provides:
 * - `Angle` — tracks shaft position across multiple revolutions using a
 *   zero-crossing detection algorithm on successive raw encoder reads.
 * - `AbsoluteEncoder` — abstract base class for absolute position sensors.
 *   Concrete implementations (e.g., `SPIEncoder`) override `read()`.
 */

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <imxrt.h>
#include <SPI.h>
#include <wiring.h>
#include "helpers.hpp"
#include "discrete_filter.hpp"


// ===========================================================================
// Angle
// ===========================================================================

/**
 * @brief Multi-turn shaft angle with overflow tracking.
 *
 * @details Wraps a raw single-turn encoder reading (0–2π) into a continuous
 * multi-turn representation. On each call to `update_angle()`, the delta
 * from the previous reading is checked against a ±π threshold: if the jump
 * exceeds π radians, a rotation boundary has been crossed and `rotations` is
 * incremented or decremented accordingly.
 *
 * `direction` inverts the sign of all returned angles, allowing mechanical
 * installation in either orientation without changing control code.
 *
 * @note `update_angle()` silently ignores negative inputs (used to discard
 *       error returns from `SPIEncoder::read()` when the glitch filter
 *       rejects a frame).
 */
struct Angle
{
  int   rotations = 0;   ///< Cumulative full-revolution count (signed).
  float radians   = 0.f; ///< Current fractional position within one revolution [rad, 0–2π].
  int   direction = 1;   ///< +1 or −1; multiplied into all returned angles.

  /**
   * @brief Returns the total signed multi-turn angle.
   *
   * @returns @f$ \text{direction} \times (2\pi \cdot \text{rotations} + \text{radians}) @f$ [rad].
   */
  float get_full_angle() const
  {
    return static_cast<float>(direction) * (static_cast<float>(rotations) * _2_PI_ + radians);
  }

  /**
   * @brief Returns the signed single-turn angle (ignores rotation count).
   *
   * @returns @f$ \text{direction} \times \text{radians} @f$ [rad, in (−2π, 2π)].
   */
  float get_angle() const
  {
    return static_cast<float>(direction) * radians;
  }

  /**
   * @brief Updates angle state from a new raw single-turn reading.
   *
   * @details Detects rotation crossings by comparing the delta against ±π.
   * A delta > +π means the encoder wrapped downwards (e.g., 6.2 → 0.1),
   * so `rotations` is decremented; a delta < −π means it wrapped upwards,
   * so `rotations` is incremented.
   *
   * @param new_radians New raw single-turn angle [rad, 0–2π].
   *                    Negative values (e.g., −1 from the glitch filter) are
   *                    silently discarded.
   */
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

  /**
   * @brief Resets rotation count and angle to zero.
   *
   * @note Call this at the start of a control session to establish a reference
   *       position. The encoder will track relative displacement from this point.
   */
  void reset() {
    radians = 0.f;
    rotations = 0;
  }
};


// ===========================================================================
// AbsoluteEncoder
// ===========================================================================

/**
 * @brief Abstract interface for single-turn absolute position encoders.
 *
 * @details Derived classes implement the hardware-specific read protocol.
 * The `BrushlessController` holds a pointer to an `AbsoluteEncoder` so that
 * different encoder types (SPI, SSI, BISS-C, etc.) can be used without
 * changing controller code.
 *
 * @note `read()` is expected to be called once per control cycle (100 µs at
 *       10 kHz). Implementations must complete within the ISR budget.
 */
class AbsoluteEncoder
{
public:
  AbsoluteEncoder() = default;
  ~AbsoluteEncoder() = default;

  /**
   * @brief Reads the current shaft angle.
   * @returns Shaft angle [rad] in the range [0, 2π), or a negative sentinel
   *          value on read error (protocol-specific).
   */
  virtual float read();
};

#endif // ENCODER_HPP
