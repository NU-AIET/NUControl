/**
 * @file driver.hpp
 * @brief Three-phase and two-phase motor gate driver wrappers.
 *
 * @details Provides `BrushlessDriver` (three-phase inverter) and `BrushedDriver`
 * (two-phase H-bridge) classes that abstract PWM generation and driver enable/disable
 * for motor control applications on Teensy 4.x (iMXRT1062).
 *
 * PWM is initially configured via Arduino's `analogWriteFrequency()` /
 * `analogWriteResolution()` / `analogWrite()` API. After `init()`,
 * `BrushlessDriver::configure_center_aligned()` may be called to switch to
 * center-aligned (up-down) counting mode for noise-free ADC sampling — see
 * `pwm_hal.hpp` for the underlying mechanism.
 */

#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <Arduino.h>
#include "transformations.hpp"
#include "pwm_hal.hpp"


// ===========================================================================
// BrushlessDriver
// ===========================================================================

/**
 * @brief Three-phase brushless motor gate driver with PWM control.
 *
 * @details Manages three PWM output pins (phases A, B, C) and one digital
 * enable pin. Converts voltage commands (V) to PWM duty counts and writes them
 * via `analogWrite()`. After `configure_center_aligned()`, the FlexPWM
 * peripheral switches to center-aligned mode and the duty values continue to
 * be set correctly by subsequent `analogWrite()` calls.
 *
 * ## Typical usage
 * @code
 * BrushlessDriver driver{{9, 8, 7}, 6, 20000.f, 12, 24.f};
 * driver.init();
 * driver.configure_center_aligned();  // optional; enables DMA-timed ADC
 * driver.enable();
 * driver.set_phase_voltages({1.5f, 0.f, -1.5f});
 * @endcode
 *
 * @note `configure_center_aligned()` must be called **after** `init()` because
 * `init()` calls `analogWriteFrequency()`, which establishes the VAL1 counter
 * modulus that `configure_center_aligned()` then halves.
 */
class BrushlessDriver
{
public:
  BrushlessDriver() = default;
  ~BrushlessDriver() = default;

  /**
   * @brief Constructs a three-phase gate driver.
   *
   * @param pins         PWM pins for phases A, B, and C respectively.
   * @param enable       Digital pin connected to the driver enable input (HIGH = on).
   * @param PWM_freq     Desired PWM carrier frequency in Hz. Default 20 000 Hz.
   *                     After `configure_center_aligned()`, the actual FlexPWM
   *                     frequency remains at this value (VAL1 is halved).
   * @param PWM_res      PWM resolution in bits [8, 12]. Default 8.
   *                     Determines the `MAX_PWM` duty range: `1 << PWM_res`.
   * @param driver_volts High-side supply voltage in volts (e.g., 24 V).
   * @param max_voltage  Software-imposed voltage ceiling ≤ `driver_volts`.
   */
  BrushlessDriver(
    const PhaseValues<int> pins, int enable, float PWM_freq = 20000.f, int PWM_res = 8,
    float driver_volts = 24.f, float max_voltage = 24.f)
  : pins_(pins),
    enable_pin_(enable),
    PWM_freq_(PWM_freq),
    PWM_resolution_(PWM_res),
    MAX_PWM_(1 << PWM_res),
    driver_voltage_(driver_volts),
    MAX_VOLT_(min(driver_volts, max_voltage))
  {}

  /**
   * @brief Initialises all pins, sets PWM frequency and resolution, and disables the driver.
   *
   * @details Configures all three phase pins as `OUTPUT`, calls
   * `analogWriteFrequency()` on each (which sets FlexPWM VAL1), and calls
   * `analogWriteResolution()`. Zeroes all phase voltages and pulls the enable
   * pin LOW.
   *
   * @returns `true` if initialisation completed (always true currently; kept for
   *          future error detection e.g., pin validation).
   *
   * @note Call this before `configure_center_aligned()`. The order matters: this
   *       function establishes the FlexPWM VAL1 modulus that the center-aligned
   *       configuration reads and halves.
   */
  bool init()
  {
    pinMode(pins_.a, OUTPUT);
    pinMode(pins_.b, OUTPUT);
    pinMode(pins_.c, OUTPUT);

    pinMode(enable_pin_, OUTPUT);

    analogWriteFrequency(pins_.a, PWM_freq_);
    analogWriteFrequency(pins_.b, PWM_freq_);
    analogWriteFrequency(pins_.c, PWM_freq_);

    analogWriteResolution(PWM_resolution_);

    // Ensure everything is at 0
    set_phase_voltages({0.f, 0.f, 0.f});

    disable();
    inited_ = true;
    return inited_;
  }

  /**
   * @brief Switches all three phase PWM outputs to center-aligned (up-down) mode.
   *
   * @details For each of the three phase pins:
   * 1. Looks up the FlexPWM module, submodule, and channel via `pwm_hal::get_flexpwm_pin()`.
   * 2. Calls `pwm_hal::configure_center_aligned()`, which halves VAL1 (to preserve
   *    the configured PWM frequency), scales VAL3/VAL5 proportionally, sets VAL0
   *    to the new VAL1 (the counter peak), and sets the CTRL FULL bit.
   * 3. Enables the VAL0 output trigger on phase A's submodule only — this
   *    submodule becomes the ADC trigger source.
   * 4. Configures intra-module SYNC when phase B and C share the same FlexPWM
   *    module as phase A (not possible across modules without XBAR).
   *
   * After this call, `get_trigger_source()` returns the FlexPWM descriptor for
   * the trigger-output submodule (phase A pin), which is passed to
   * `adc_dma_hal::connect_flexpwm_trigger_to_adcN()`.
   *
   * @returns `true` if all three pins were found in the FlexPWM pin table.
   *          `false` if any pin is not a recognised PWM pin (the FlexPWM table
   *          returns `{nullptr, 0, 0}` for that entry).
   *
   * @warning Must be called after `init()`. Calling before `init()` reads
   *          uninitialised VAL1 values, producing incorrect frequency scaling.
   *
   * @warning Do NOT call `analogWriteFrequency()` on any of the three phase pins
   *          after this function, as that would reset VAL1 to the edge-aligned
   *          value and halve the actual PWM frequency.
   *
   * @note Pins on different FlexPWM modules (e.g., pin 9 on FLEXPWM2 and pins
   *       7, 8 on FLEXPWM1) cannot be hardware-synced within this function.
   *       They run at the same frequency from the same 150 MHz clock source
   *       and maintain near-zero phase offset in practice, but strict synchrony
   *       requires XBAR-based inter-module routing (see `pwm_hal.hpp`).
   *
   * @todo Implement XBAR-based inter-module SYNC for strict three-phase alignment
   *       when phase pins span multiple FlexPWM modules.
   */
  bool configure_center_aligned()
  {
    if (!inited_) { return false; }

    pwm_hal::FlexPWMPin fpins[3] = {
      pwm_hal::get_flexpwm_pin(static_cast<uint8_t>(pins_.a)),
      pwm_hal::get_flexpwm_pin(static_cast<uint8_t>(pins_.b)),
      pwm_hal::get_flexpwm_pin(static_cast<uint8_t>(pins_.c))
    };

    for (auto & fp : fpins) {
      if (fp.module == nullptr) { return false; }
      pwm_hal::configure_center_aligned(fp);
    }

    // Enable ADC trigger output on phase A submodule only.
    // Phase A (pins_.a) is designated as the trigger source for the ADC chain.
    pwm_hal::configure_trigger_output(fpins[0]);

    // Configure intra-module SYNC if phase B or C share a module with phase A.
    // Slaving them to phase A's submodule ensures they reach their counter peak
    // at the same instant, maximising the quiet ADC sampling window.
    if (fpins[1].module == fpins[0].module && fpins[1].submodule != fpins[0].submodule) {
      pwm_hal::configure_submodule_sync(fpins[0].module, fpins[0].submodule, fpins[1].submodule);
    }
    if (fpins[2].module == fpins[0].module && fpins[2].submodule != fpins[0].submodule) {
      pwm_hal::configure_submodule_sync(fpins[0].module, fpins[0].submodule, fpins[2].submodule);
    }

    // Store trigger source for caller retrieval
    trigger_source_ = fpins[0];
    center_aligned_ = true;
    return true;
  }

  /**
   * @brief Returns the FlexPWM descriptor of the ADC trigger source.
   *
   * @details After `configure_center_aligned()`, the phase-A submodule has
   * its TCTRL trigger output enabled. This function exposes that descriptor
   * so `BrushlessController::configure_adc_trigger()` can pass it to the
   * `adc_dma_hal` XBAR routing functions.
   *
   * @returns FlexPWMPin for phase A. `module == nullptr` if
   *          `configure_center_aligned()` has not been called or failed.
   */
  pwm_hal::FlexPWMPin get_trigger_source() const { return trigger_source_; }

  /// @brief Returns `true` if center-aligned PWM has been successfully configured.
  bool is_center_aligned() const { return center_aligned_; }

  /**
   * @brief Enables the gate driver by asserting the enable pin HIGH.
   * @note No voltage is applied until `set_phase_voltages()` is called.
   */
  void enable()
  {
    digitalWriteFast(enable_pin_, HIGH);
    enabled_ = true;
  }

  /**
   * @brief Disables the gate driver.
   *
   * @details Zeroes all phase voltages **before** pulling the enable pin LOW.
   * This ordering prevents a brief shoot-through glitch that could occur if
   * the enable pin dropped while a phase was at a non-zero duty cycle.
   */
  void disable()
  {
    set_phase_voltages({0.f, 0.f, 0.f});
    digitalWriteFast(enable_pin_, LOW);
    enabled_ = false;
  }

  /**
   * @brief Applies three-phase voltages to the motor by writing PWM duty cycles.
   *
   * @details Converts each voltage to a PWM duty count via `volts_to_PWM()` and
   * writes it via `analogWrite()`. Returns immediately with `{0,0,0}` if the
   * driver is not enabled.
   *
   * After `configure_center_aligned()`, `analogWrite()` scales the duty count
   * correctly against the halved VAL1 modulus, so the percentage duty and the
   * output voltage relationship are preserved.
   *
   * @param voltages Phase voltages in volts {Va, Vb, Vc}. Values are clamped to
   *                 [0, `MAX_VOLT_`] before conversion; negative voltages are
   *                 clamped to 0 (no regenerative braking path through this driver).
   * @returns Applied PWM duty counts {a, b, c} in the range [0, MAX_PWM-1],
   *          or {0,0,0} if the driver is not enabled.
   */
  PhaseValues<int> set_phase_voltages(PhaseValues<float> voltages) const
  {
    if (!enabled_) {
      return {0, 0, 0};
    }
    const PhaseValues<int> duty = PhaseValues<int>{
      volts_to_PWM(voltages.a),
      volts_to_PWM(voltages.b),
      volts_to_PWM(voltages.c)
    };
    analogWrite(pins_.a, duty.a);
    analogWrite(pins_.b, duty.b);
    analogWrite(pins_.c, duty.c);

    return duty;
  }

private:
  const PhaseValues<int> pins_;    ///< PWM pin numbers for phases A, B, C

  const int enable_pin_;           ///< Digital enable pin (HIGH = driver on)

  bool enabled_ = false;           ///< True if the driver is currently enabled
  bool inited_  = false;           ///< True after `init()` completes

  const float PWM_freq_;           ///< Configured PWM frequency [Hz]
  const int   PWM_resolution_;     ///< PWM resolution [bits]
  const int   MAX_PWM_;            ///< Maximum duty count = 1 << PWM_resolution_

  const float driver_voltage_;     ///< High-side supply voltage [V]
  const float MAX_VOLT_;           ///< Software voltage ceiling [V] ≤ driver_voltage_

  bool center_aligned_ = false;    ///< True after configure_center_aligned() succeeds
  pwm_hal::FlexPWMPin trigger_source_{nullptr, 0, 0}; ///< ADC trigger source descriptor

  /**
   * @brief Converts a phase voltage to a PWM duty count.
   *
   * @details Implements:
   * @f[ \text{duty} = \text{round}\!\left(
   *   \frac{\text{clamp}(V, 0, V_{\max})}{V_{\text{bus}}} \times (2^{\text{res}} - 1)
   * \f]
   *
   * The clamp enforces [0, MAX_VOLT_]; negative voltages map to 0 (duty count 0).
   * The result is rounded to the nearest integer to minimise quantisation error.
   *
   * @param volt Phase voltage in volts.
   * @returns Duty cycle count in [0, MAX_PWM_ - 1].
   */
  int volts_to_PWM(float volt) const
  {
    if (driver_voltage_ <= 0.f) { return 0; }
    int v =
      static_cast<int>(std::round(
        std::clamp<float>(
          volt, 0,
          MAX_VOLT_) / driver_voltage_ * static_cast<float>(MAX_PWM_ - 1)));
    return v;
  }

};


// ===========================================================================
// BrushedDriver
// ===========================================================================

/**
 * @brief Two-phase brushed DC motor H-bridge driver.
 *
 * @details Controls two PWM pins (high and low side) and a single enable pin.
 * Voltage commands are converted to PWM duty cycles using the same
 * `volts_to_PWM()` algorithm as `BrushlessDriver`.
 *
 * @note This class is partially implemented (no `configure_center_aligned()`
 * support). It is retained for future use with brushed actuators.
 *
 * @todo Add deadtime compensation: reduce effective duty by `t_dead × f_pwm`
 * counts near 0% and 100% to prevent H-bridge shoot-through if the gate
 * driver IC does not handle deadtime insertion internally.
 */
class BrushedDriver
{

public:
  BrushedDriver() = default;
  ~BrushedDriver() = default;

  /**
   * @brief Constructs a two-phase brushed motor driver.
   *
   * @param pins         A `std::pair<int,int>` of PWM pin numbers (high, low).
   * @param enable       Enable pin (HIGH = on).
   * @param PWM_freq     PWM frequency [Hz]. Default 200 000 Hz (200 kHz).
   * @param PWM_res      PWM resolution [bits]. Default 8.
   * @param driver_volts High-side supply voltage [V].
   * @param max_voltage  Software voltage ceiling [V].
   */
  BrushedDriver(
    const std::pair<int, int> pins, int enable, float PWM_freq = 200000.f, int PWM_res = 8,
    float driver_volts = 24.f, float max_voltage = 24.f)
  : pins_(pins),
    enable_pin_(enable),
    PWM_freq_(PWM_freq),
    PWM_resolution_(PWM_res),
    MAX_PWM_(1 << PWM_res),
    driver_voltage_(driver_volts),
    MAX_VOLT_(min(driver_volts, max_voltage))
  {}

  /**
   * @brief Initialises pins, PWM frequency/resolution, and disables the driver.
   * @returns `true` always.
   */
  bool init()
  {
    pinMode(pins_.first, OUTPUT);
    pinMode(pins_.second, OUTPUT);

    pinMode(enable_pin_, OUTPUT);

    analogWriteFrequency(pins_.first, PWM_freq_);
    analogWriteFrequency(pins_.second, PWM_freq_);

    analogWriteResolution(PWM_resolution_);

    // Ensure everything is at 0
    set_voltage({0.f, 0.f});

    disable();
    inited_ = true;
    return inited_;
  }

  /// @brief Enables the H-bridge.
  void enable()
  {
    digitalWriteFast(enable_pin_, HIGH);
    enabled_ = true;
  }

  /**
   * @brief Disables the H-bridge.
   * @details Zeroes both channels before pulling enable LOW.
   */
  void disable()
  {
    set_voltage({0.f, 0.f});
    digitalWriteFast(enable_pin_, LOW);
    enabled_ = false;
  }

  /**
   * @brief Applies a two-phase voltage command.
   *
   * @param volts Pair of {high-side voltage, low-side voltage} in volts.
   * @returns Applied duty counts as a `std::pair<float, float>` (note: float,
   *          kept from original implementation for compatibility).
   */
  std::pair<float, float> set_voltage(std::pair<float, float> volts) const
  {

    const std::pair<float, float> duty = std::pair<float, float>{
      volts_to_PWM(volts.first),
      volts_to_PWM(volts.second),
    };
    analogWrite(pins_.first, duty.first);
    analogWrite(pins_.second, duty.second);

    return duty;
  }

private:
  const std::pair<int, int> pins_;

  const int enable_pin_;

  bool enabled_ = false;
  bool inited_  = false;

  const float PWM_freq_;
  const int   PWM_resolution_;
  const int   MAX_PWM_;

  const float driver_voltage_;
  const float MAX_VOLT_;

  /**
   * @brief Converts a voltage to a PWM duty count. See `BrushlessDriver::volts_to_PWM()`.
   * @param volt Phase voltage [V].
   * @returns Duty count in [0, MAX_PWM_ - 1].
   */
  int volts_to_PWM(float volt) const
  {
    if (driver_voltage_ <= 0.f) { return 0; }
    int v =
      static_cast<int>(std::round(
        std::clamp<float>(
          volt, 0,
          MAX_VOLT_) / driver_voltage_ * static_cast<float>(MAX_PWM_ - 1)));
    return v;
  }


};

#endif // DRIVER_HPP
