/**
 * @file current_sense.hpp
 * @brief Inline current sensing for three-phase motor control.
 *
 * @details Provides `InlineCurrentSensor` (single analog channel → Amps) and
 * `InlineCurrentSensorPackage` (2–3 sensors mapped to motor phases A/B/C via
 * automated calibration).
 *
 * ## Read paths
 * Two read paths are available:
 * - **Software path** (`read()` / `read_filtered()`): calls `analogRead()` — blocking,
 *   ~3–5 µs per call, sample time is undefined relative to the PWM cycle.
 * - **DMA path** (`read_dma()`): reads from a `volatile uint16_t` buffer that is
 *   updated automatically by a DMA channel on each FlexPWM counter-TOP trigger.
 *   Non-blocking, ~200 ns, sample is always timed to the PWM quiet window.
 *
 * Enable the DMA path by calling `InlineCurrentSensorPackage::init_dma_sensors()`
 * after `BrushlessDriver::configure_center_aligned()` has been called and
 * `adc_dma_hal` has been configured. The switch is transparent: `get_phase_currents()`
 * dispatches to the DMA path automatically once enabled.
 *
 * ## Sensor calibration
 * `align_sensors()` identifies which sensor measures which motor phase, and in which
 * direction, by injecting a small voltage into each phase sequentially and observing
 * the sensor responses. This must be run once at startup (or replaced by
 * `load_calibration()` with saved values).
 *
 * ## Two-sensor operation
 * If only two sensors are available, the third phase current is reconstructed from
 * Kirchhoff's Current Law:
 * @f[ I_{\text{missing}} = -I_a - I_b @f]
 * (The sum of currents entering the motor neutral point is zero for a star-connected winding.)
 */

#ifndef CURRENT_SENSE_HPP
#define CURRENT_SENSE_HPP

#include <array>
#include <vector>
#include <Arduino.h>
#include <DMAChannel.h>
#include "driver.hpp"
#include "helpers.hpp"
#include "errors.hpp"
#include "discrete_filter.hpp"
#include "adc_dma_hal.hpp"

// ===========================================================================
// InlineCurrentSensor
// ===========================================================================

/**
 * @brief Single analog current sensor on one motor phase.
 *
 * @details Reads an analog pin (via `analogRead()` or DMA), subtracts a
 * mid-rail DC offset (typically 1.65 V), and scales to Amps using a gain
 * derived from the sensor hardware:
 * @f[ I = G \cdot (V_{\text{ADC}} - V_{\text{offset}}) @f]
 * where @f$ G @f$ is `amps_per_volt` and @f$ V_{\text{ADC}} = \text{reading} \times
 * (3.3\,\text{V} / 2^{\text{res}}) @f$.
 *
 * Saturation warnings are issued when the magnitude exceeds 1.5× the gain;
 * an error callback fires at 2× the gain.
 */
class InlineCurrentSensor
{
public:
  InlineCurrentSensor() = default;
  ~InlineCurrentSensor() = default;

  /// Fraction of gain at which a saturation warning is logged (150 % of full-scale).
  static constexpr float SATURATE_FACTOR = 1.5f;
  /// Fraction of gain at which the error callback is invoked (200 % of full-scale).
  static constexpr float MAX_FACTOR      = 2.0f;
  /// Offset validation: deviation from ideal that triggers an ERROR [V].
  static constexpr float OFFSET_FAIL_V   = 0.5f;
  /// Offset validation: deviation from ideal that triggers a WARN [V].
  static constexpr float OFFSET_WARN_V   = 0.05f;

  /**
   * @brief Constructs a current sensor with a direct amps-per-volt gain.
   *
   * @param pin          Arduino analog pin number (e.g., `A4`).
   * @param amps_per_volt Gain [A/V] — the sensor output voltage per amp of
   *                      phase current. Typical value: 5 A/V.
   * @param ADC_res      ADC resolution in bits [8, 12]. Default 10.
   *                     Sets `ADC_GAIN_ = 3.3V / (1 << ADC_res)`.
   * @param f            Error callback invoked when current exceeds 2× gain.
   *                     Defaults to `handle_errors()` which halts execution.
   *
   * @note The saturation threshold (1.5× gain) corresponds to a sensor output
   *       voltage of `1.5 / gain` V above or below the 1.65 V mid-rail, which
   *       is approximately the full-scale range of a typical inline sensor.
   */
  InlineCurrentSensor(
    int pin, float amps_per_volt, int ADC_res = 10, void(*f) (
      ErrorCodes) = * handle_errors)
  : pin_(pin),
    gain_(amps_per_volt),
    SATURATE_READING_(SATURATE_FACTOR * gain_),
    MAX_READING_(MAX_FACTOR * gain_),
    ADC_GAIN_(3.3f / (1 << ADC_res)),
    error_callback(f)
  {
    analogReadRes(ADC_res);
  }

  /**
   * @brief Constructs a current sensor from shunt resistance and op-amp gain.
   *
   * @details Computes `amps_per_volt = 1 / (shunt_resistance_ohms * op_amp_gain)`.
   *
   * @param pin                 Arduino analog pin number.
   * @param shunt_resistance_ohms Shunt resistor value [Ω].
   * @param op_amp_gain         Transimpedance amplifier voltage gain [V/V].
   * @param ADC_res             ADC resolution in bits.
   */
  InlineCurrentSensor(int pin, float shunt_resistance_ohms, float op_amp_gain, int ADC_res = 10)
  : pin_(pin),
    gain_(1.f / (shunt_resistance_ohms * op_amp_gain)),
    SATURATE_READING_(SATURATE_FACTOR * gain_),
    MAX_READING_(MAX_FACTOR * gain_),
    ADC_GAIN_(3.3f / (1 << ADC_res))
  {
    analogReadRes(ADC_res);
  }

  /**
   * @brief Initialises the pin and validates the DC offset.
   *
   * @details Sets the pin to `INPUT` mode and calls `validate_offset()` to
   * confirm the sensor's quiescent output is within ±0.5 V of 1.65 V.
   *
   * @returns `true` if offset is within tolerance, `false` otherwise.
   *
   * @warning `validate_offset()` is blocking: it samples the pin 10 000 times
   *          via `analogRead()`, which takes approximately 30–50 ms per sensor
   *          at typical Teensy ADC speeds. Do not call from an ISR.
   */
  bool init_sensor()
  {
    pinMode(pin_, INPUT);
    return validate_offset();
  }

  /**
   * @brief Reads instantaneous phase current via software `analogRead()`.
   *
   * @details Performs a single blocking `analogRead()`, converts to volts,
   * subtracts the calibrated offset, and multiplies by gain:
   * @f[ I = G \cdot (\text{reading} \times \frac{3.3}{2^{\text{res}}} - V_{\text{offset}}) @f]
   *
   * Issues a `Serial.println` warning if `|I| > SATURATE_READING_` and invokes
   * the error callback if `|I| > MAX_READING_`.
   *
   * @returns Phase current in Amperes. Positive = current flowing into the phase.
   *
   * @warning Blocking (~3–5 µs). Do not use inside the control ISR once the DMA
   *          path has been initialised; use `read_dma()` instead.
   */
  float read() const
  {
    auto amps = gain_ * (analogRead(pin_) * ADC_GAIN_ - offset_);
    if (fabs(amps) > SATURATE_READING_) {
      nu_log::push(nu_log::Level::WARN, nu_log::Id::CURRENT_SATURATED, amps);
      if (fabs(amps) > MAX_READING_) {
        nu_log::push(nu_log::Level::ERROR, nu_log::Id::CURRENT_OVER_LIMIT, amps);
        error_callback(ErrorCodes::CURRENT_SENSE_OVER_LIMIT);
      }
    }
    return amps;
  }

  /**
   * @brief Assigns the discrete filter used by `read_filtered()`.
   * @param filter A `DiscreteFilter<float,float>` instance (e.g., `Butterworth2nd`).
   */
  void set_filter(DiscreteFilter<float, float> filter)
  {
    filter_ = filter;
  }

  /**
   * @brief Reads and low-pass filters the phase current (software path).
   * @returns Filtered phase current in Amperes.
   */
  float read_filtered()
  {
    return filter_.update(read());
  }

  // -------------------------------------------------------------------------
  // DMA path
  // -------------------------------------------------------------------------

  /**
   * @brief Initialises the DMA channel for this sensor (package-level setup must
   *        be done first).
   *
   * @details Sets up the `DMAChannel` to transfer from `ADC1_R0` (or `ADC2_R0`)
   * to `dma_buf_` on the completion of ADC_ETC TRIG[`trig_idx`]. The ADC_ETC
   * trigger configuration and the ADC hardware-trigger mode setup are performed at
   * the package level by `InlineCurrentSensorPackage::init_dma_sensors()` before
   * this function is called.
   *
   * The `trig_idx` parameter selects the DMAMUX hardware event source:
   * - `trig_idx = 0` → `DMAMUX_SOURCE_ADC_ETC + 0` (sensor 0 in the package)
   * - `trig_idx = 1` → `DMAMUX_SOURCE_ADC_ETC + 1` (sensor 1)
   * - `trig_idx = 2` → `DMAMUX_SOURCE_ADC_ETC + 2` (sensor 2)
   *
   * @param adc_channel iMXRT1062 ADC channel number (NOT Arduino pin alias).
   *                    Use `adc_dma_hal::arduino_pin_to_adc1_channel()` to convert.
   * @param trig_idx    ADC_ETC trigger index for this sensor [0, 2]. Must match
   *                    the index used in `configure_adc_etc_trig()` and
   *                    `connect_flexpwm_trigger_to_adc1_trig()` for this sensor.
   * @param use_adc2    `false` → DMA reads `ADC1_R0` (default, for pins A0–A8).
   *                    `true`  → DMA reads `ADC2_R0` (for ADC2-only pins, e.g., A9).
   * @returns `true` if the DMA channel was configured successfully.
   *
   * @warning Must be called **after** `init_sensor()` (blocking `analogRead()`
   *          offset calibration). Calling before `init_sensor()` will race with
   *          ADC_SC2 being reset to software-trigger mode by `analogRead()`.
   *
   * @warning Do NOT call `analogRead()` on the same ADC channel after this
   *          function returns; `analogRead()` resets `ADC_SC2` (software trigger).
   */
  bool init_dma(uint8_t adc_channel, uint8_t trig_idx = 0, bool use_adc2 = false)
  {
    adc_channel_  = adc_channel;
    use_adc2_     = use_adc2;
    dma_enabled_  = false;

    bool ok;
    if (!use_adc2) {
      ok = adc_dma_hal::configure_dma_for_adc1_trig(dma_ch_, dma_buf_, trig_idx);
    } else {
      ok = adc_dma_hal::configure_dma_for_adc2(dma_ch_, dma_buf_);
    }

    dma_enabled_ = ok;
    return ok;
  }

  /**
   * @brief Reads phase current from the DMA result buffer (non-blocking).
   *
   * @details Returns the current value using the same conversion formula as `read()`,
   * but reads from `dma_buf_` (written by the DMA channel) instead of calling
   * `analogRead()`. Executes in ~200 ns (one memory read + arithmetic).
   *
   * @returns Phase current in Amperes. Returns 0 if `init_dma()` has not been called.
   *
   * @note The returned value is always the most recent completed conversion.
   *       The update rate equals the FlexPWM trigger frequency (one per PWM period).
   *       No filtering is applied here; call `read_filtered_dma()` for filtered data.
   */
  float read_dma() const
  {
    if (!dma_enabled_) { return 0.f; }
    // dma_buf_ holds the raw ADC result (same format as analogRead() output)
    auto amps = gain_ * (static_cast<float>(dma_buf_) * ADC_GAIN_ - offset_);
    if (fabs(amps) > SATURATE_READING_) {
      nu_log::push(nu_log::Level::WARN, nu_log::Id::CURRENT_SATURATED, amps);
      if (fabs(amps) > MAX_READING_) {
        nu_log::push(nu_log::Level::ERROR, nu_log::Id::CURRENT_OVER_LIMIT, amps);
        error_callback(ErrorCodes::CURRENT_SENSE_OVER_LIMIT);
      }
    }
    return amps;
  }

  /**
   * @brief Reads and filters the DMA-sourced current.
   * @returns Filtered phase current in Amperes via the same `DiscreteFilter`
   *          used by `read_filtered()`.
   */
  float read_filtered_dma()
  {
    return filter_.update(read_dma());
  }

  /// @brief Returns `true` if the DMA path has been successfully initialised.
  bool is_dma_enabled() const { return dma_enabled_; }

  /// @brief Returns the Arduino analog pin number used by this sensor.
  int get_pin() const { return pin_; }

  /**
   * @brief Reads this sensor directly via `analogRead()`, bypassing alignment.
   *
   * @details Identical to `read()` but public and clearly named for use in
   * pre-calibration routines (e.g., `BrushlessController::identify_rl()`)
   * where the phase-to-sensor mapping is not yet known.
   *
   * @returns Phase current in Amperes using the same gain/offset as `read()`.
   *
   * @warning Do not use this inside the control ISR — it is blocking (~3–5 µs).
   */
  float read_raw() const { return read(); }

private:
  const int   pin_;                ///< Arduino analog pin number
  const float gain_;               ///< Sensor gain [A/V]
  float       offset_ = 1.65f;    ///< DC bias at zero current [V] (mid-rail = 3.3V/2)
  const float SATURATE_READING_;  ///< Saturation threshold [A] = 1.5 × gain
  const float MAX_READING_;       ///< Over-limit threshold [A] = 2.0 × gain
  const float ADC_GAIN_;          ///< ADC LSB voltage: 3.3V / (1 << ADC_res) [V/count]

  DiscreteFilter<float, float> filter_; ///< Low-pass filter for `read_filtered()`

  void (* error_callback) (ErrorCodes); ///< Invoked when current exceeds MAX_READING_

  // DMA path members — must have persistent (global/static) lifetime
  DMAChannel       dma_ch_;              ///< DMA channel (one per sensor)
  volatile uint16_t dma_buf_ = 0;       ///< DMA destination buffer (ADC raw count)
  uint8_t          adc_channel_ = 255;  ///< iMXRT1062 ADC channel index; 255 = unset
  bool             use_adc2_    = false;///< True if this sensor uses ADC2
  bool             dma_enabled_ = false;///< True after init_dma() succeeds

  /**
   * @brief Measures and validates the sensor's quiescent (zero-current) DC offset.
   *
   * @details Averages 10 000 `analogRead()` samples. Expects a result near 1.65 V
   * (half of the 3.3 V supply, which is the mid-rail output of a bipolar current
   * sensor). Acceptance thresholds:
   * - Within ±0.05 V: ideal; uses default 1.65 V offset.
   * - Within ±0.5 V: acceptable; updates `offset_` to the measured value.
   * - Beyond ±0.5 V: sensor not responding correctly; returns `false`.
   *
   * @param n Number of samples to average. Default 10 000.
   * @returns `true` if offset is within the ±0.5 V acceptance window.
   *
   * @warning Blocking: takes approximately 30–50 ms at typical Teensy ADC speeds.
   *          Do NOT call from an ISR.
   *
   * @todo Replace with a single hardware-averaged conversion (64 samples via
   *       `ADC_CFG1_AVGE` + `ADC_SC3_AVGS`) to reduce init time from ~50 ms
   *       per sensor to under 1 ms.
   */
  bool validate_offset(size_t n = 10000)
  {
    int sum_ = 0;
    for (size_t i = 0; i < n; ++i) {
      sum_ += analogRead(pin_);
    }
    float offset = static_cast<float>(sum_) / n * ADC_GAIN_;

    if (fabs(offset_ - offset) > OFFSET_FAIL_V) {
      nu_log::push(nu_log::Level::ERROR, nu_log::Id::CS_OFFSET_FAIL, offset);
      nu_log::drain(Serial);
      return false;
    }
    if (fabs(offset_ - offset) > OFFSET_WARN_V) {
      nu_log::push(nu_log::Level::WARN, nu_log::Id::CS_OFFSET_DEVIATION, offset);
      nu_log::drain(Serial);
      offset_ = offset;
      return true;
    }
    return true;

  }

};


// ===========================================================================
// InlineCurrentSensorPackage
// ===========================================================================

/**
 * @brief A package of 2–3 `InlineCurrentSensor` instances mapped to motor phases.
 *
 * @details Manages sensor-to-phase assignment calibration and provides a unified
 * `get_phase_currents()` interface that reconstructs three-phase currents regardless
 * of how many sensors are physically installed.
 *
 * ## Sensor alignment
 * Call `align_sensors()` once at startup to:
 * 1. Inject 0.5 V into phase A (B, C) and observe which sensor responds most strongly.
 * 2. Record the sensor index and direction (±1) for each phase.
 * 3. Store in `phase_idx_` and `phase_dirs_`.
 *
 * Alternatively, call `load_calibration()` with pre-measured values to skip the
 * alignment procedure.
 *
 * ## KCL reconstruction (2-sensor mode)
 * When only two sensors are installed, the third phase current is inferred from
 * Kirchhoff's Current Law at the motor star point:
 * @f[ I_{\text{missing}} = -I_a - I_b @f]
 * This is valid for balanced (star-connected, no neutral current) three-phase loads.
 */
class InlineCurrentSensorPackage
{
public:
  InlineCurrentSensorPackage() = default;
  ~InlineCurrentSensorPackage() = default;

  /**
   * @brief Constructs a sensor package from a list of sensor pointers.
   *
   * @param sensors Vector of pointers to `InlineCurrentSensor` objects.
   *                Must contain exactly 2 or 3 sensors; fewer sensors will
   *                cause a diagnostic print but no hard failure.
   */
  InlineCurrentSensorPackage(std::vector<InlineCurrentSensor *> sensors)
  : sensors_(sensors),
    num_sensors_(sensors_.size())
  {
    nu_log::push(nu_log::Level::INFO, nu_log::Id::CS_SENSOR_COUNT,
                 static_cast<float>(num_sensors_));
    nu_log::drain(Serial);
    if (num_sensors_ < 2) {
      return;
    }
  }

  /**
   * @brief Initialises all sensors (validates DC offsets).
   *
   * @details Calls `InlineCurrentSensor::init_sensor()` on each sensor in the
   * package. All sensors must pass offset validation for this to return `true`.
   *
   * @returns `true` if all sensors initialised successfully.
   *
   * @warning Blocking: see `InlineCurrentSensor::validate_offset()`.
   */
  bool init_sensors()
  {
    bool all_inited = true;
    for (size_t i = 0; i < num_sensors_; ++i) {
      all_inited &= sensors_.at(i)->init_sensor();
    }
    return all_inited;
  }

  /// @brief Pushes calibrated phase-index and phase-direction values to the log.
  void print_calibration()
  {
    nu_log::push(nu_log::Level::INFO, nu_log::Id::CAL_CS_PHASE_IDX_A,
                 static_cast<float>(phase_idx_.a));
    nu_log::push(nu_log::Level::INFO, nu_log::Id::CAL_CS_PHASE_IDX_B,
                 static_cast<float>(phase_idx_.b));
    nu_log::push(nu_log::Level::INFO, nu_log::Id::CAL_CS_PHASE_IDX_C,
                 static_cast<float>(phase_idx_.c));
    nu_log::push(nu_log::Level::INFO, nu_log::Id::CAL_CS_PHASE_DIR_A,
                 static_cast<float>(phase_dirs_.a));
    nu_log::push(nu_log::Level::INFO, nu_log::Id::CAL_CS_PHASE_DIR_B,
                 static_cast<float>(phase_dirs_.b));
    nu_log::push(nu_log::Level::INFO, nu_log::Id::CAL_CS_PHASE_DIR_C,
                 static_cast<float>(phase_dirs_.c));
  }

  /**
   * @brief Calibrates sensor-to-phase mapping by injecting test voltages.
   *
   * @details For each motor phase (A, B, C):
   * 1. Applies `align_volts` to that phase via `driver.set_phase_voltages()`.
   * 2. Waits 100 ms for the current to settle.
   * 3. Reads all sensors and records the response.
   *
   * For each sensor, the phase that produced the largest absolute response is
   * assigned to that sensor. The sign of the response determines `phase_dirs_`:
   * +1 if the sensor reads positive when the phase is excited, −1 otherwise.
   *
   * @param driver      Reference to the `BrushlessDriver` used to inject test voltages.
   * @param align_volts Alignment voltage [V]. Defaults to 0.5 V.
   *                    Should be large enough to produce a detectable current
   *                    (> 0.05 A) but small enough to be safe. Typically
   *                    `0.5 × phase_R × SAFE_CURRENT`.
   * @returns `true` if all sensors produced a response > 0.05 A, `false` otherwise.
   *
   * @note The driver is briefly enabled during alignment and disabled at the end.
   */
  bool align_sensors(BrushlessDriver & driver, float align_volts = 0.5f)
  {

    driver.enable();

    driver.set_phase_voltages({align_volts, 0, 0});
    delay(100);
    auto reads_a = read_sensors();
    driver.set_phase_voltages({0, 0, 0});
    delay(100);

    driver.set_phase_voltages({0, align_volts, 0});
    delay(100);
    auto reads_b = read_sensors();
    driver.set_phase_voltages({0, 0, 0});
    delay(100);

    driver.set_phase_voltages({0, 0, align_volts});
    delay(100);
    auto reads_c = read_sensors();
    driver.set_phase_voltages({0, 0, 0});

    driver.disable();

    std::array<PhaseValues<float>, 3> sensor_readings;

    for (size_t i = 0; i < num_sensors_; ++i) {
      nu_log::push(nu_log::Level::INFO, nu_log::Id::CS_ALIGN_SENSOR_IDX,
                   static_cast<float>(i));
      nu_log::push(nu_log::Level::INFO, nu_log::Id::CS_ALIGN_READ_A, reads_a.at(i));
      nu_log::push(nu_log::Level::INFO, nu_log::Id::CS_ALIGN_READ_B, reads_b.at(i));
      nu_log::push(nu_log::Level::INFO, nu_log::Id::CS_ALIGN_READ_C, reads_c.at(i));

      sensor_readings.at(i) = {reads_a.at(i), reads_b.at(i), reads_c.at(i)};
    }

    for (size_t i = 0; i < num_sensors_; ++i) {
      const auto max_ =
        max(
        fabs(sensor_readings.at(i).a),
        max(fabs(sensor_readings.at(i).b), fabs(sensor_readings.at(i).c)));

      if (max_ < 0.05f) {
        nu_log::push(nu_log::Level::ERROR, nu_log::Id::CS_NO_CURRENT,
                     static_cast<float>(i));
        nu_log::drain(Serial);
        return false;
      }

      if (max_ == fabs(sensor_readings.at(i).a)) {
        phase_idx_.a = i;
        if (max_ > sensor_readings.at(i).a) {
          phase_dirs_.a = -1;
        } else {
          phase_dirs_.a = 1;
        }
        continue;
      }

      if (max_ == fabs(sensor_readings.at(i).b)) {
        phase_idx_.b = i;
        if (max_ > sensor_readings.at(i).b) {
          phase_dirs_.b = -1;
        } else {
          phase_dirs_.b = 1;
        }
        continue;
      }

      if (max_ == fabs(sensor_readings.at(i).c)) {
        phase_idx_.c = i;
        if (max_ > sensor_readings.at(i).c) {
          phase_dirs_.c = -1;
        } else {
          phase_dirs_.c = 1;
        }
        continue;
      }
    }

    print_calibration();
    nu_log::drain(Serial);

    aligned_ = true;
    return aligned_;

  }

  /**
   * @brief Loads pre-measured calibration data, bypassing `align_sensors()`.
   *
   * @param phase_idx  Sensor-to-phase index map: `{sensor_idx_for_A, ..._B, ..._C}`.
   *                   Set to -1 for unmeasured phases.
   * @param phase_dirs Phase direction signs: `{dir_A, dir_B, dir_C}` ∈ {+1, −1, 0}.
   * @returns `true` always; sets `aligned_ = true`.
   */
  bool load_calibration(PhaseValues<int> phase_idx, PhaseValues<int> phase_dirs)
  {
    phase_idx_ = phase_idx;
    phase_dirs_ = phase_dirs;
    print_calibration();
    aligned_ = true;
    return aligned_;
  }

  /**
   * @brief Returns three-phase currents using the calibrated sensor mapping.
   *
   * @details Reads all sensors (software or DMA path), maps sensor indices to
   * phases using `phase_idx_`, applies direction signs via `phase_dirs_`, and
   * reconstructs the missing phase via KCL if only two sensors are present.
   *
   * Dispatch order:
   * 1. DMA path (`dma_mode_ == true`): reads from `dma_buf_` in each sensor.
   * 2. Filtered software path (`filter == true`): `analogRead()` + discrete filter.
   * 3. Raw software path: unfiltered `analogRead()`.
   *
   * @param filter If `true` (and DMA mode is off), applies the configured
   *               `DiscreteFilter` to each raw reading.
   * @returns `{I_a, I_b, I_c}` in Amperes. Returns `{0,0,0}` if sensors have
   *          not been aligned.
   *
   * @note When `dma_mode_` is active, the `filter` parameter is ignored for the
   *       raw DMA read. Filtered DMA reads are available by calling
   *       `read_filtered_dma_sensors()` directly.
   */
  PhaseValues<float> get_phase_currents(bool filter = true)
  {
    if (!aligned_) {
      nu_log::push(nu_log::Level::WARN, nu_log::Id::CS_NOT_ALIGNED);
      return {};
    }

    PhaseValues<float> phase_amps;

    std::array<float, 3> amps{0.f, 0.f, 0.f};

    if (dma_mode_) {
      amps = read_dma_sensors();
    } else if (filter) {
      amps = read_filtered_sensors();
    } else {
      amps = read_sensors();
    }

    if (phase_idx_.a > -1) {
      phase_amps.a = phase_dirs_.a * amps.at(phase_idx_.a);
    }
    if (phase_idx_.b > -1) {
      phase_amps.b = phase_dirs_.b * amps.at(phase_idx_.b);
    }
    if (phase_idx_.c > -1) {
      phase_amps.c = phase_dirs_.c * amps.at(phase_idx_.c);
    }

    if (num_sensors_ == 3) {
      // Assume no one would have more than 3 sensors
      return phase_amps;
    }

    // KCL reconstruction: I_missing = -I_a - I_b (sum at neutral point = 0)
    if (phase_idx_.a == -1) {
      phase_amps.a = -phase_amps.b - phase_amps.c;
    }
    if (phase_idx_.b == -1) {
      phase_amps.b = -phase_amps.a - phase_amps.c;
    }
    if (phase_idx_.c == -1) {
      phase_amps.c = -phase_amps.a - phase_amps.b;
    }

    return phase_amps;
  }

  /**
   * @brief Assigns the same discrete filter to all sensors in the package.
   * @param filter Filter instance (e.g., `Butterworth2nd<float>`).
   */
  void set_filters(DiscreteFilter<float, float> filter)
  {
    for (size_t i = 0; i < num_sensors_; ++i) {
      sensors_.at(i)->set_filter(filter);
    }
  }

  // -------------------------------------------------------------------------
  // DMA path
  // -------------------------------------------------------------------------

  /**
   * @brief Initialises DMA-backed reads for all sensors in the package.
   *
   * @details Performs the complete per-sensor DMA initialisation sequence for
   * every sensor in the package:
   * 1. Reads each sensor's Arduino pin via `get_pin()`.
   * 2. Converts the pin to an iMXRT1062 ADC channel number (ADC1 or ADC2).
   * 3. Configures ADC1 hardware-trigger mode (once, for all ADC1 sensors).
   * 4. For each sensor: routes the FlexPWM trigger through XBAR to
   *    `ADC1_ETC_TRIG[i]` (or `ADC2_ETC_TRIG0` for ADC2 sensors).
   * 5. Configures `ADC_ETC_TRIG[i]` for a single-channel conversion.
   * 6. Calls `sensor->init_dma(channel, trig_idx=i, use_adc2)` to arm the DMA
   *    channel for that sensor.
   *
   * After this call, `get_phase_currents()` automatically dispatches to
   * `read_dma_sensors()` (non-blocking, ~200 ns per sensor vs ~5 µs for
   * `analogRead()`).
   *
   * ## Multi-sensor ADC scheduling
   * Sensors are assigned ADC_ETC trigger indices 0, 1, 2 in order. Since ADC1
   * has a single conversion core, triggers on the same ADC are serialised by
   * ADC_ETC (queued internally). Each conversion takes ~0.5 µs at 12-bit with
   * ADLSMP, so a 3-sensor package adds ~1.5 µs of total latency from the
   * FlexPWM trigger edge to all three DMA buffers being updated.
   *
   * Pins that map exclusively to ADC2 (currently only A9 = ADC2 channel 0) are
   * automatically detected and routed through `ADC2_ETC_TRIG0` instead.
   *
   * @param trigger_source FlexPWMPin from `BrushlessDriver::get_trigger_source()`.
   *                       The XBAR1 routing from this FlexPWM submodule to each
   *                       ADC_ETC trigger input is configured here.
   * @returns `true` if all sensors in the package were configured successfully.
   *          `false` if any sensor's pin does not map to a valid ADC channel.
   *
   * @warning Must be called **after** `init_sensors()` (which uses `analogRead()`
   *          for offset calibration). Calling before `init_sensors()` will cause
   *          `analogRead()` to reset `ADC_SC2`, silently disabling the DMA path.
   *
   * @warning Do NOT call `analogRead()` on any sensor pin after this function
   *          returns. Doing so resets `ADC_SC2 = 0` (software trigger mode).
   */
  /**
   * @brief Reads all sensors in physical order without requiring phase alignment.
   *
   * @details Returns raw `analogRead()` current values indexed by sensor number
   * (not by motor phase).  Bypasses the `aligned_` guard in `get_phase_currents()`.
   * Intended exclusively for pre-calibration routines such as
   * `BrushlessController::identify_rl()` where the sensor-to-phase mapping has
   * not yet been established.
   *
   * @returns `std::array<float, 3>` of current readings [A], indexed by sensor
   *          position.  Positions beyond `num_sensors_` are zero.
   *
   * @warning Blocking (~3–5 µs per sensor). Do not call from the control ISR.
   * @warning Do not call after `init_dma_sensors()` — `analogRead()` resets
   *          the ADC to software-trigger mode, silently disabling the DMA path.
   */
  std::array<float, 3> read_unaligned() const { return read_sensors(); }

  bool init_dma_sensors(const pwm_hal::FlexPWMPin & trigger_source)
  {
    bool all_ok = true;
    bool adc1_hw_trigger_configured = false;

    for (uint8_t i = 0; i < static_cast<uint8_t>(num_sensors_); ++i) {
      const int arduino_pin = sensors_.at(i)->get_pin();

      // Resolve pin to ADC channel
      const uint8_t ch1 = adc_dma_hal::arduino_pin_to_adc1_channel(
          static_cast<uint8_t>(arduino_pin));
      const uint8_t ch2 = adc_dma_hal::arduino_pin_to_adc2_channel(
          static_cast<uint8_t>(arduino_pin));

      const bool use_adc2   = (ch1 == 255u && ch2 != 255u);
      const uint8_t adc_ch  = use_adc2 ? ch2 : ch1;

      if (adc_ch == 255u) {
        nu_log::push(nu_log::Level::WARN, nu_log::Id::CS_DMA_PIN_INVALID,
                     static_cast<float>(arduino_pin));
        nu_log::drain(Serial);
        all_ok = false;
        continue;
      }

      if (!use_adc2) {
        // Configure ADC1 hardware-trigger mode once (shared by all ADC1 sensors)
        if (!adc1_hw_trigger_configured) {
          adc_dma_hal::configure_adc1_hardware_trigger(adc_ch);
          adc1_hw_trigger_configured = true;
        }
        // Route FlexPWM trigger → XBAR → ADC1_ETC_TRIG[i]
        adc_dma_hal::connect_flexpwm_trigger_to_adc1_trig(trigger_source, i);
        // Configure ADC_ETC TRIG[i] for this channel
        adc_dma_hal::configure_adc_etc_trig(i, adc_ch);
      } else {
        // ADC2 sensor (e.g., A9): use existing ADC2 path with TRIG1 in ADC_ETC
        adc_dma_hal::configure_adc2_hardware_trigger(adc_ch);
        adc_dma_hal::connect_flexpwm_trigger_to_adc2(trigger_source);
        adc_dma_hal::configure_adc_etc_trigger2(adc_ch);
      }

      // Arm the DMA channel for this sensor
      all_ok &= sensors_.at(i)->init_dma(adc_ch, i, use_adc2);
    }

    dma_mode_ = all_ok;
    return all_ok;
  }

private:
  std::vector<InlineCurrentSensor *> sensors_; ///< Pointers to the physical sensors
  size_t num_sensors_;                          ///< Number of sensors (2 or 3)
  PhaseValues<int> phase_idx_{-1, -1, -1};     ///< sensor index for each phase; -1 = unmeasured
  PhaseValues<int> phase_dirs_{0, 0, 0};        ///< direction sign (+1 or -1) per phase

  bool aligned_  = false; ///< True after align_sensors() or load_calibration() succeeds
  bool dma_mode_ = false; ///< True after init_dma_sensors() succeeds

  /**
   * @brief Reads all sensors once via `analogRead()` (no filter).
   *
   * @details Returns up to 3 readings in a fixed-size array. Positions beyond
   * `num_sensors_` are zero-initialised.
   *
   * @returns Array of raw current readings [A], indexed by sensor number.
   *
   * @note Changed from `std::vector<float>` to `std::array<float,3>` to eliminate
   *       heap allocation inside the control ISR (called every 100 µs).
   *
   * @todo Replace with a stack-allocated `std::array<float, N>` templated on
   *       sensor count when a `constexpr` sensor count is available.
   */
  std::array<float, 3> read_sensors() const
  {
    std::array<float, 3> reads{0.f, 0.f, 0.f};
    for (size_t i = 0; i < num_sensors_; ++i) {
      reads.at(i) = sensors_.at(i)->read();
    }
    return reads;
  }

  /**
   * @brief Reads all sensors via `analogRead()` and applies the configured filter.
   * @returns Filtered current readings [A], indexed by sensor number.
   */
  std::array<float, 3> read_filtered_sensors()
  {
    std::array<float, 3> reads{0.f, 0.f, 0.f};
    for (size_t i = 0; i < num_sensors_; ++i) {
      reads.at(i) = sensors_.at(i)->read_filtered();
    }
    return reads;
  }

  /**
   * @brief Reads all sensors from DMA buffers (non-blocking).
   * @returns DMA-sourced current readings [A], indexed by sensor number.
   */
  std::array<float, 3> read_dma_sensors()
  {
    std::array<float, 3> reads{0.f, 0.f, 0.f};
    for (size_t i = 0; i < num_sensors_; ++i) {
      reads.at(i) = sensors_.at(i)->read_dma();
    }
    return reads;
  }

};

#endif // CURRENT_SENSE_HPP
