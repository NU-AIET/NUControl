/**
 * @file thermal_model.hpp
 * @brief Temperature sensor abstraction and two-node winding thermal model
 *        for motor current derating.
 *
 * @details Four components are provided:
 *
 * ### `TemperatureSensor` (abstract)
 * Virtual base class with a single `read_celsius()` method.  Concrete
 * subclasses can represent NTC thermistors, I²C/SPI digital sensors, or
 * any other measurement source.  The thermal model holds a raw pointer
 * to this base, so lifetime management is the caller's responsibility.
 *
 * ### `NTCThermistor`
 * Reads an NTC thermistor wired as a voltage divider on an Arduino analog
 * pin (R_series pull-up from V_supply to ADC pin, NTC from ADC pin to GND).
 * Uses the B-value (beta-coefficient) equation:
 * @f[
 *   T = \frac{1}{\frac{\ln(R/R_0)}{B} + \frac{1}{T_0}}  \quad [\text{K}]
 * @f]
 * The conversion is done in `refresh()` (blocking, call from the main loop);
 * `read_celsius()` returns the cached result and is ISR-safe.
 *
 * ### `DMANTCThermistor`
 * Identical B-value conversion but reads from a `volatile uint16_t` buffer
 * that is kept current by a background DMA transfer configured via
 * `adc_dma_hal.hpp`.  `read_celsius()` is fully ISR-safe and non-blocking;
 * no `refresh()` call is needed.  This is the preferred implementation when
 * a thermistor is mounted in a thermally active location (e.g., on the motor
 * windings) where updates at the full control rate are desirable.
 *
 * ### `TwoNodeThermalModel`
 * Implements a first-order lumped-parameter thermal network:
 * @code
 *   I²R heat
 *      ↓
 *  [winding] ──Rth_wc──> [case/housing] ──Rth_ca──> [ambient]
 *   Cth_w                  Cth_c
 * @endcode
 * Difference equations (forward Euler, safe at control-loop rates):
 * @f[
 *   \dot{T}_w = \frac{P - (T_w - T_c)/R_{wc}}{C_{w}}
 * @f]
 * @f[
 *   \dot{T}_c = \frac{(T_w - T_c)/R_{wc} - (T_c - T_{amb})/R_{ca}}{C_{c}}
 * @f]
 *
 * **Optional physical sensors for winding and case nodes:**
 * A `TemperatureSensor` can be attached to either or both nodes via
 * `set_winding_sensor()` / `set_case_sensor()`.  When a sensor is present,
 * its reading replaces the ODE prediction for that node each tick — the
 * sensor measurement is treated as ground truth.  Nodes without a sensor
 * continue to be driven by the ODE, using the best available (potentially
 * sensed) estimate of adjacent nodes for heat-flow calculations.
 *
 * Current derating begins when @f$ T_w @f$ reaches 75 % of the thermal
 * headroom above ambient (configurable), and scales linearly to zero at
 * @f$ T_{max} @f$:
 * @f[
 *   I_{limit} = I_{max} \cdot \frac{T_{max} - T_w}{T_{max} - T_{warn}}
 *   \quad \text{for } T_w \in [T_{warn},\, T_{max}]
 * @f]
 *
 * @note The thermal model runs at the control-loop rate (typically 10 kHz).
 *       Forward Euler is stable when the control period is much shorter than
 *       the smallest thermal time constant (@f$ R_{wc} \cdot C_w @f$).
 *       For motor windings this is typically several seconds, so 100 µs steps
 *       are well within the stability margin.
 *
 * @note All `TemperatureSensor` pointers held by `TwoNodeThermalModel` are
 *       non-owning; the sensor objects must outlive the model.  All sensors
 *       passed to the model **must** return from `read_celsius()` without
 *       blocking — use `DMANTCThermistor` or the `NTCThermistor` cache pattern.
 *
 * @see motors.hpp for `MotorParameters` thermal fields that parameterise the model.
 * @see brushless_controller.hpp for integration points (`set_thermal_model()`,
 *      `enable_thermal_limiting()`).
 * @see adc_dma_hal.hpp for DMA ADC setup required by `DMANTCThermistor`.
 */

#ifndef THERMAL_MODEL_HPP
#define THERMAL_MODEL_HPP

#include <Arduino.h>
#include <math.h>

// ===========================================================================
// TemperatureSensor
// ===========================================================================

/**
 * @brief Abstract interface for a temperature sensor.
 *
 * @details Provides a single virtual method `read_celsius()` that returns
 * the current temperature reading.  Concrete implementations can be:
 * - NTC thermistor on an analog pin (`NTCThermistor`)
 * - Onboard Teensy temperature sensor (override with `InternalTempSensor`)
 * - I²C/SPI digital sensors (override with a caching read in the subclass)
 *
 * @note If the sensor read is slow (I²C, UART), implement a caching strategy
 *       in the derived class and refresh the cache from a lower-priority task
 *       or a periodic timer, returning the cached value from `read_celsius()`.
 *       The thermal model calls `read_celsius()` once per control tick.
 */
class TemperatureSensor
{
public:
  virtual ~TemperatureSensor() = default;

  /**
   * @brief Returns the current temperature measurement.
   * @returns Temperature in degrees Celsius.
   */
  virtual float read_celsius() = 0;
};


// ===========================================================================
// NTCThermistor
// ===========================================================================

/**
 * @brief NTC thermistor temperature sensor using the B-value equation.
 *
 * @details Assumes a voltage-divider circuit with:
 * - Pull-up resistor `R_series` from `V_supply` to the ADC pin.
 * - NTC thermistor from the ADC pin to GND.
 *
 * @code
 * V_supply ──[R_series]──┬── ADC pin
 *                        │
 *                      [NTC]
 *                        │
 *                       GND
 * @endcode
 *
 * ADC voltage → NTC resistance:
 * @f[ R_{ntc} = R_{series} \cdot \frac{V_{adc}}{V_{supply} - V_{adc}} @f]
 *
 * B-value equation (NTC resistance → temperature):
 * @f[ T [K] = \frac{1}{\dfrac{\ln(R_{ntc}/R_0)}{B} + \dfrac{1}{T_0}} @f]
 *
 * @note `T_nominal_c` and `R_nominal` are the sensor's calibration point
 *       (typically 25 °C / 10 kΩ from the datasheet).
 *
 * @warning `refresh()` performs a blocking `analogRead()` — call it only from
 *          a non-ISR context (e.g., the main loop).  `read_celsius()` is
 *          ISR-safe and returns the most-recently cached value.
 *
 * @todo For higher accuracy, replace the B-value equation with the three-parameter
 *       Steinhart-Hart model: @f$ 1/T = A + B\ln R + C(\ln R)^3 @f$.
 */
class NTCThermistor : public TemperatureSensor
{
public:
  NTCThermistor() = default;
  ~NTCThermistor() = default;

  /**
   * @brief Constructs an NTC thermistor sensor.
   *
   * @param pin          Arduino analog pin the voltage divider output is connected to.
   * @param R_series     Pull-up resistor value [Ω].
   * @param R_nominal    Thermistor resistance at `T_nominal_c` [Ω] (e.g., 10 000 Ω).
   * @param T_nominal_c  Calibration temperature [°C] (e.g., 25 °C).
   * @param beta         B-value (beta coefficient) [K] from the thermistor datasheet.
   *                     Typical range: 3000–4500 K.
   * @param V_supply     Supply voltage of the voltage divider [V]. Default 3.3 V.
   * @param adc_bits     ADC resolution in bits (must match `analogReadResolution()`).
   *                     Default 12.
   */
  NTCThermistor(int pin, float R_series, float R_nominal, float T_nominal_c,
                float beta, float V_supply = 3.3f, int adc_bits = 12)
  : pin_(pin),
    R_series_(R_series),
    R_nominal_(R_nominal),
    T_nominal_k_(T_nominal_c + 273.15f),
    beta_(beta),
    V_supply_(V_supply),
    adc_max_(static_cast<float>((1 << adc_bits) - 1)),
    cached_celsius_(T_nominal_c)
  {}

  /**
   * @brief Performs a blocking ADC read and updates the cached temperature.
   *
   * @details Call this periodically from the main loop (or a low-priority timer)
   * at whatever rate is sufficient for ambient temperature tracking (1–10 Hz is
   * typical).  The result is stored in an internal cache that `read_celsius()`
   * returns without blocking.
   *
   * @note Clamps the result to [−40, 200] °C to guard against open/short circuits.
   *
   * @warning Blocking (~3–5 µs for a single `analogRead()`).
   *          Do NOT call from the control ISR.
   */
  void refresh()
  {
    float adc_raw = static_cast<float>(analogRead(pin_));

    // Guard against divide-by-zero: clamp away from rail values
    adc_raw = constrain(adc_raw, 1.f, adc_max_ - 1.f);

    float V_adc = adc_raw / adc_max_ * V_supply_;
    float R_ntc = R_series_ * V_adc / (V_supply_ - V_adc);

    // B-value equation → temperature in Kelvin
    float T_k = 1.f / (logf(R_ntc / R_nominal_) / beta_ + 1.f / T_nominal_k_);

    cached_celsius_ = constrain(T_k - 273.15f, -40.f, 200.f);
  }

  /**
   * @brief Returns the most recently cached temperature [°C].
   *
   * @details ISR-safe and non-blocking.  The returned value is only as fresh
   * as the last call to `refresh()`.  Call `refresh()` from the main loop at
   * the desired ambient-update rate before starting the control loop, and
   * periodically thereafter.
   *
   * @returns Cached temperature in degrees Celsius.
   */
  float read_celsius() override { return cached_celsius_; }

private:
  int   pin_;              ///< Arduino analog pin
  float R_series_;         ///< Pull-up resistor [Ω]
  float R_nominal_;        ///< NTC nominal resistance at T_nominal_k_ [Ω]
  float T_nominal_k_;      ///< NTC nominal temperature [K]
  float beta_;             ///< B-value / beta coefficient [K]
  float V_supply_;         ///< Supply voltage [V]
  float adc_max_;          ///< ADC full-scale count (2^bits - 1)
  float cached_celsius_;   ///< Last reading from refresh() [°C]
};


// ===========================================================================
// DMANTCThermistor
// ===========================================================================

/**
 * @brief NTC thermistor backed by a DMA-populated ADC result buffer.
 *
 * @details Uses the same voltage-divider circuit and B-value equation as
 * `NTCThermistor`, but reads from a `volatile uint16_t` that is written
 * continuously by a background DMA transfer configured via `adc_dma_hal.hpp`.
 * `read_celsius()` is fully **ISR-safe and non-blocking** — it performs only
 * arithmetic on the latest DMA result; no `refresh()` call is ever needed.
 *
 * ## Typical setup
 * @code
 * // 1. Declare a global DMA destination buffer (must outlive the sensor):
 * volatile uint16_t ntc_dma_buf = 0;
 *
 * // 2. During init, configure ADC + DMA (adc_dma_hal.hpp):
 * adc_dma_hal::configure_dma_for_adc1(ntc_dma_buf, A9_ADC_CHANNEL);
 *
 * // 3. Construct the sensor:
 * DMANTCThermistor winding_ntc(ntc_dma_buf,
 *   10'000.f,  // R_series [Ω]
 *   10'000.f,  // R_nominal [Ω] at 25 °C
 *   25.f,      // T_nominal [°C]
 *   3950.f,    // beta [K]
 *   3.3f, 12);
 *
 * // 4. Pass to thermal model — no further maintenance required:
 * model.set_winding_sensor(&winding_ntc);
 * @endcode
 *
 * @note DMA ADC accuracy depends on consistent ADC configuration.  If other
 *       code calls `analogRead()` on unrelated pins it may reset ADC_SC2 and
 *       disable the hardware trigger; see the warning in `adc_dma_hal.hpp`.
 *
 * @todo For higher accuracy, replace the B-value equation with the three-parameter
 *       Steinhart-Hart model: @f$ 1/T = A + B\ln R + C(\ln R)^3 @f$.
 */
class DMANTCThermistor : public TemperatureSensor
{
public:
  DMANTCThermistor() = delete;
  ~DMANTCThermistor() = default;

  /**
   * @brief Constructs a DMA-backed NTC thermistor sensor.
   *
   * @param dma_result   Reference to the `volatile uint16_t` DMA destination
   *                     buffer.  Must be a global or static variable; do NOT
   *                     pass a stack-allocated buffer (the DMA outlives the frame).
   * @param R_series     Pull-up resistor value [Ω].
   * @param R_nominal    Thermistor resistance at `T_nominal_c` [Ω] (e.g., 10 000 Ω).
   * @param T_nominal_c  Calibration temperature [°C] (e.g., 25 °C).
   * @param beta         B-value (beta coefficient) [K] from the thermistor datasheet.
   * @param V_supply     Supply voltage of the voltage divider [V]. Default 3.3 V.
   * @param adc_bits     ADC resolution in bits. Must match the DMA ADC configuration.
   *                     Default 12.
   */
  DMANTCThermistor(const volatile uint16_t & dma_result,
                   float R_series, float R_nominal, float T_nominal_c,
                   float beta, float V_supply = 3.3f, int adc_bits = 12)
  : dma_result_(dma_result),
    R_series_(R_series),
    R_nominal_(R_nominal),
    T_nominal_k_(T_nominal_c + 273.15f),
    beta_(beta),
    V_supply_(V_supply),
    adc_max_(static_cast<float>((1 << adc_bits) - 1))
  {}

  /**
   * @brief Converts the latest DMA ADC result to °C and returns it.
   *
   * @details ISR-safe and non-blocking.  Reads one `volatile uint16_t` (a
   * single 16-bit aligned load on Cortex-M7, which is atomic) then performs
   * the B-value conversion entirely in registers.
   *
   * @returns Temperature in degrees Celsius, clamped to [−40, 200] °C.
   */
  float read_celsius() override
  {
    // Single volatile load — atomic on ARM for aligned 16-bit access.
    float adc_raw = constrain(static_cast<float>(dma_result_), 1.f, adc_max_ - 1.f);

    float V_adc = adc_raw / adc_max_ * V_supply_;
    float R_ntc = R_series_ * V_adc / (V_supply_ - V_adc);

    float T_k = 1.f / (logf(R_ntc / R_nominal_) / beta_ + 1.f / T_nominal_k_);
    return constrain(T_k - 273.15f, -40.f, 200.f);
  }

private:
  const volatile uint16_t & dma_result_; ///< Reference to DMA-populated ADC result
  float R_series_;     ///< Pull-up resistor [Ω]
  float R_nominal_;    ///< NTC nominal resistance at T_nominal_k_ [Ω]
  float T_nominal_k_;  ///< NTC nominal temperature [K]
  float beta_;         ///< B-value / beta coefficient [K]
  float V_supply_;     ///< Supply voltage [V]
  float adc_max_;      ///< ADC full-scale count (2^bits - 1)
};


// ===========================================================================
// TwoNodeThermalModel
// ===========================================================================

/**
 * @brief Two-node (winding → case → ambient) lumped-parameter thermal model.
 *
 * @details Tracks two temperature states per control tick:
 * - @f$ T_w @f$: winding (copper) temperature [°C]
 * - @f$ T_c @f$: case / housing temperature [°C]
 *
 * The ambient temperature is either read from an optional `TemperatureSensor`
 * (polled once per `update()` call) or held at a user-specified fixed value.
 *
 * ## Parameters
 * All four thermal parameters come from `MotorParameters` (see `motors.hpp`):
 *
 * | Parameter            | Symbol       | Typical range         |
 * |----------------------|--------------|-----------------------|
 * | `R_th_winding_case`  | @f$ R_{wc} @f$ | 0.5–5 °C/W          |
 * | `C_th_winding`       | @f$ C_w @f$  | 5–50 J/°C            |
 * | `R_th_case_ambient`  | @f$ R_{ca} @f$ | 1–20 °C/W          |
 * | `C_th_case`          | @f$ C_c @f$  | 50–500 J/°C          |
 *
 * ## Derating curve
 * The derating window starts at `T_warn = T_amb + 0.75 * (T_max - T_amb)`:
 *
 * @code
 * I_limit
 *  I_max ─────────────────┐
 *                          \
 *                           \
 *     0 ─────────────────────┴──── T_winding
 *                         T_warn  T_max
 * @endcode
 *
 * ## Physical sensors for model nodes
 * Attach a `TemperatureSensor` to the winding or case node via
 * `set_winding_sensor()` / `set_case_sensor()`.  On each `update()` call:
 * - The ODE integrator runs using the best available estimate for each node.
 * - If a sensor is attached to a node, the sensor reading **replaces** the
 *   ODE prediction for that node — the physical measurement is ground truth.
 * - Nodes without sensors continue to be integrated normally, with heat-flow
 *   computations driven by any sensed neighbours.
 *
 * All sensors must return from `read_celsius()` without blocking.  Use
 * `DMANTCThermistor` for sensors that update at the control rate, or
 * `NTCThermistor` with `refresh()` called from the main loop for ambient-rate
 * updates (1–10 Hz is sufficient for slowly-changing nodes like the case).
 *
 * ## Thread safety
 * `T_winding_` and `T_case_` are updated by `update()` (called from the
 * control ISR) and read by `get_winding_temperature()` / `get_case_temperature()`
 * from the main loop.  Both are declared `volatile` to prevent the compiler
 * from caching them in registers across the ISR boundary.  ARM Cortex-M7
 * aligned 32-bit stores are single-instruction, so occasional stale reads
 * from the main loop are acceptable for monitoring purposes.
 * Do not enable/disable the model from inside an ISR.
 */
class TwoNodeThermalModel
{
public:
  TwoNodeThermalModel() = default;
  ~TwoNodeThermalModel() = default;

  /**
   * @brief Constructs the thermal model from explicit thermal parameters.
   *
   * @param R_th_winding_case  Thermal resistance winding → case [°C/W].
   * @param C_th_winding       Thermal capacitance of winding [J/°C].
   * @param R_th_case_ambient  Thermal resistance case → ambient [°C/W].
   * @param C_th_case          Thermal capacitance of case [J/°C].
   * @param T_max_winding      Maximum safe winding temperature [°C].
   *                           Derating reaches zero at this value.
   * @param ambient_sensor     Optional sensor for live ambient temperature.
   *                           Pass `nullptr` (default) to use the fixed ambient
   *                           set by `set_ambient_temperature()`.
   * @param winding_sensor     Optional sensor physically mounted on the motor
   *                           windings.  When provided, its reading replaces
   *                           the ODE estimate for @f$ T_w @f$ every tick.
   *                           Must be ISR-safe (non-blocking `read_celsius()`).
   * @param case_sensor        Optional sensor mounted on the motor case/housing.
   *                           When provided, its reading replaces the ODE
   *                           estimate for @f$ T_c @f$ every tick.
   *                           Must be ISR-safe (non-blocking `read_celsius()`).
   */
  TwoNodeThermalModel(float R_th_winding_case, float C_th_winding,
                      float R_th_case_ambient, float C_th_case,
                      float T_max_winding,
                      TemperatureSensor * ambient_sensor  = nullptr,
                      TemperatureSensor * winding_sensor  = nullptr,
                      TemperatureSensor * case_sensor     = nullptr)
  : R_th_wc_(R_th_winding_case),
    C_th_w_(C_th_winding),
    R_th_ca_(R_th_case_ambient),
    C_th_c_(C_th_case),
    T_max_(T_max_winding),
    ambient_sensor_(ambient_sensor),
    winding_sensor_(winding_sensor),
    case_sensor_(case_sensor)
  {}

  /**
   * @brief Resets the thermal state to known initial conditions.
   *
   * @details Call before starting the control loop so the model begins from a
   * known condition rather than the default 25 °C.  If physical sensors are
   * attached to the winding or case nodes, their current readings are used to
   * seed those states (ensure `NTCThermistor::refresh()` has been called first
   * if using the blocking variant).  Nodes without sensors are initialised to
   * `T_ambient_c`.
   *
   * @param T_ambient_c Starting temperature for unsensed nodes [°C]. Default 25 °C.
   */
  void reset(float T_ambient_c = 25.f)
  {
    T_ambient_cached_ = T_ambient_c;
    T_winding_ = winding_sensor_ ? winding_sensor_->read_celsius() : T_ambient_c;
    T_case_    = case_sensor_    ? case_sensor_->read_celsius()    : T_ambient_c;
  }

  /**
   * @brief Advances the thermal model by one control step.
   *
   * @details Should be called once per control tick from
   * `BrushlessController::update_control()`.  The per-phase dissipated power is:
   * @f[
   *   P = \frac{1}{2} (I_q^2 + I_d^2) \cdot R_{phase}
   * @f]
   * (factor of 1/2 converts amplitude-invariant Park-frame peak current to
   * per-phase RMS power).  The caller is responsible for computing and
   * passing `P_winding_watts`.
   *
   * @param P_winding_watts  Per-phase I²R heat dissipation [W] this tick.
   * @param dt_s             Control period [s].
   */
  void update(float P_winding_watts, float dt_s)
  {
    // Poll ambient sensor if attached.  read_celsius() MUST be non-blocking
    // (ISR-safe).  For NTCThermistor, call refresh() from the main loop and
    // pass the NTCThermistor pointer here — read_celsius() returns the cache.
    if (ambient_sensor_ != nullptr) {
      T_ambient_cached_ = ambient_sensor_->read_celsius();
    }

    // Use sensor readings as the current node temperatures when available;
    // otherwise fall back to the ODE state.  Loading volatile members into
    // locals avoids repeated volatile reads in the arithmetic below.
    const float T_w = winding_sensor_ ? winding_sensor_->read_celsius()
                                      : static_cast<float>(T_winding_);
    const float T_c = case_sensor_    ? case_sensor_->read_celsius()
                                      : static_cast<float>(T_case_);

    // Heat flows computed using best available node temperatures.
    const float Q_out_w = (T_w - T_c)              / R_th_wc_;  // winding → case
    const float Q_out_c = (T_c - T_ambient_cached_) / R_th_ca_; // case → ambient

    // Sensor-attached nodes: store sensor reading (ground truth).
    // Unsensed nodes: advance via forward-Euler ODE.
    T_winding_ = winding_sensor_ ? T_w : T_w + (P_winding_watts - Q_out_w) / C_th_w_ * dt_s;
    T_case_    = case_sensor_    ? T_c : T_c + (Q_out_w - Q_out_c)         / C_th_c_ * dt_s;
  }

  /**
   * @brief Returns the derated current limit based on current winding temperature.
   *
   * @details The derating window spans from `T_warn` (75 % of the thermal
   * headroom above ambient) to `T_max`:
   * - @f$ T_w \le T_{warn} @f$: returns `max_current` unchanged.
   * - @f$ T_w \ge T_{max}  @f$: returns `0`.
   * - Between: linear interpolation.
   *
   * @param max_current  Nominal maximum current [A] (typically `motor_.MAX_CURRENT`).
   * @returns Derated current limit [A] in [0, max_current].
   */
  float get_current_limit(float max_current) const
  {
    float T_warn = T_ambient_cached_ + 0.75f * (T_max_ - T_ambient_cached_);
    if (T_winding_ <= T_warn) { return max_current; }
    if (T_winding_ >= T_max_) { return 0.f; }
    return max_current * (T_max_ - T_winding_) / (T_max_ - T_warn);
  }

  // -------------------------------------------------------------------------
  // Accessors
  // -------------------------------------------------------------------------

  /// @returns Estimated winding temperature [°C].
  float get_winding_temperature()  const { return T_winding_; }

  /// @returns Estimated case/housing temperature [°C].
  float get_case_temperature()     const { return T_case_; }

  /// @returns Last ambient temperature used (from sensor or fixed) [°C].
  float get_ambient_temperature()  const { return T_ambient_cached_; }

  /**
   * @brief Overrides the fixed ambient temperature used when no sensor is attached.
   * @param T_c Ambient temperature [°C].
   */
  void set_ambient_temperature(float T_c)  { T_ambient_fixed_ = T_c; T_ambient_cached_ = T_c; }

  /**
   * @brief Replaces (or removes) the live ambient temperature sensor.
   *
   * @param sensor Pointer to a `TemperatureSensor`, or `nullptr` to revert
   *               to the fixed ambient temperature.
   */
  void set_ambient_sensor(TemperatureSensor * sensor) { ambient_sensor_ = sensor; }

  /**
   * @brief Attaches (or detaches) a physical temperature sensor on the motor windings.
   *
   * @details When attached, `update()` uses this sensor's reading as the
   * winding temperature each tick instead of the ODE prediction.  The sensor
   * must return from `read_celsius()` without blocking (use `DMANTCThermistor`
   * or the `NTCThermistor` cache pattern).
   *
   * @param sensor Pointer to a `TemperatureSensor`, or `nullptr` to revert
   *               to pure ODE estimation for the winding node.
   */
  void set_winding_sensor(TemperatureSensor * sensor) { winding_sensor_ = sensor; }

  /**
   * @brief Attaches (or detaches) a physical temperature sensor on the motor case.
   *
   * @details When attached, `update()` uses this sensor's reading as the
   * case temperature each tick instead of the ODE prediction.  The sensor
   * must return from `read_celsius()` without blocking.
   *
   * @param sensor Pointer to a `TemperatureSensor`, or `nullptr` to revert
   *               to pure ODE estimation for the case node.
   */
  void set_case_sensor(TemperatureSensor * sensor) { case_sensor_ = sensor; }

  // -------------------------------------------------------------------------
  // Thermal parameter setters (useful after identify_rl() updates phase_R)
  // -------------------------------------------------------------------------

  /// @brief Updates the thermal resistance winding → case [°C/W].
  void set_R_th_winding_case(float r)  { R_th_wc_ = r; }

  /// @brief Updates the thermal resistance case → ambient [°C/W].
  void set_R_th_case_ambient(float r)  { R_th_ca_ = r; }

  /// @brief Updates the winding thermal capacitance [J/°C].
  void set_C_th_winding(float c)       { C_th_w_ = c; }

  /// @brief Updates the case thermal capacitance [J/°C].
  void set_C_th_case(float c)          { C_th_c_ = c; }

  /// @brief Updates the maximum winding temperature [°C].
  void set_T_max(float t)              { T_max_ = t; }

private:
  float R_th_wc_  = 1.f;    ///< Thermal resistance winding → case [°C/W]
  float C_th_w_   = 10.f;   ///< Thermal capacitance of winding [J/°C]
  float R_th_ca_  = 5.f;    ///< Thermal resistance case → ambient [°C/W]
  float C_th_c_   = 100.f;  ///< Thermal capacitance of case [J/°C]
  float T_max_    = 130.f;  ///< Maximum winding temperature [°C]

  volatile float T_winding_  = 25.f; ///< Estimated winding temperature state [°C] (written from ISR)
  volatile float T_case_     = 25.f; ///< Estimated case temperature state [°C] (written from ISR)

  float T_ambient_fixed_  = 25.f; ///< Fixed ambient temperature fallback [°C]
  float T_ambient_cached_ = 25.f; ///< Last ambient reading (from sensor or fixed) [°C]

  TemperatureSensor * ambient_sensor_ = nullptr;  ///< Optional live ambient sensor
  TemperatureSensor * winding_sensor_ = nullptr;  ///< Optional physical winding sensor
  TemperatureSensor * case_sensor_    = nullptr;  ///< Optional physical case/housing sensor
};


#endif // THERMAL_MODEL_HPP
