/**
 * @file brushless_controller.hpp
 * @brief Field-Oriented Control (FOC) brushless motor controller.
 *
 * @details `BrushlessController` implements a full synchronous-frame current
 * controller for three-phase brushless DC motors. The control pipeline per
 * 100 µs step is:
 *
 * @code
 * update_sensors()
 *   ├── SPI encoder read  → shaft_angle_, shaft_velocity_
 *   ├── ADC current read  → phase_currents_ → quaddirect_currents_ (Park)
 *   └── [optional] thermal model update (I²R power → winding temperature)
 *
 * update_control()
 *   ├── TORQUE mode:
 *   │   ├── [optional] anticogging torque injection
 *   │   ├── torque → desired current (I_q = T / kT)
 *   │   ├── [optional] thermal current derating
 *   │   ├── feedforward (bilinear RL model)
 *   │   ├── feedback (PI on I_qd error)
 *   │   ├── back-EMF decoupler
 *   │   ├── voltage filter
 *   │   ├── [optional] anticogging voltage injection
 *   │   └── center_phase_voltages() + 1V offset → driver
 *   └── OPEN_LOOP_VELOCITY mode (used during calibration)
 * @endcode
 *
 * ## Center-aligned PWM + DMA ADC
 * `init_components()` calls `BrushlessDriver::configure_center_aligned()` and
 * then `configure_adc_trigger()` to set up the full FlexPWM → XBAR1 → ADC_ETC
 * → ADC → DMA chain. After this, `update_sensors()` reads current from DMA
 * buffers (non-blocking, ~200 ns) instead of `analogRead()` (~5 µs).
 *
 * ## Calibration
 * Either call `align_sensors()` (automated, rotates the motor briefly) or
 * `load_calibration()` with previously saved values to skip alignment.
 */

#ifndef BRUSHLESS_CONTROLLER_HPP
#define BRUSHLESS_CONTROLLER_HPP

#include <TeensyTimerTool.h>
#include "driver.hpp"
#include "current_sense.hpp"
#include "transformations.hpp"
#include "encoder.hpp"
#include "discrete_filter.hpp"
#include "motors.hpp"
#include "adc_dma_hal.hpp"
#include "thermal_model.hpp"
// #include "anticog_helpers.hpp"
// #include <functional>

// ===========================================================================
// RLIdentResult
// ===========================================================================

/**
 * @brief Results returned by `BrushlessController::identify_rl()`.
 *
 * @details Holds the identified single-phase winding resistance and inductance,
 * plus a success flag.  Even when `success == false`, the fields are populated
 * with the raw computed values so the caller can inspect them and decide whether
 * to accept or discard the measurement.
 *
 * @see BrushlessController::identify_rl()
 */
struct RLIdentResult
{
  float phase_R = 0.f;  ///< Identified single-phase resistance [Ω].
  float phase_L = 0.f;  ///< Identified single-phase inductance [H].
  bool  success = false; ///< `true` if both values passed sanity checks.
};


// ===========================================================================
// ControllerMode
// ===========================================================================

/**
 * @brief Controller operating modes.
 *
 * @details Selected via `set_control_mode()` before `start_control()`.
 */
enum ControllerMode
{
  DISABLE,            ///< Driver disabled; no voltage applied.
  OPEN_LOOP_VELOCITY, ///< Open-loop velocity control used during calibration.
  TORQUE,             ///< Closed-loop torque / current control (normal operation).
};

/**
 * @brief Pre-measured calibration data that can be loaded without running `align_sensors()`.
 *
 * @details Avoids the ~1 s automated alignment scan at each startup. Obtain by
 * running `align_sensors()` once, calling `print_calibration()`, and recording
 * the printed values.
 */
struct BrushlessCalibration
{
  const PhaseValues<int> cs_phase_idx{-1, -1, -1};  ///< Sensor-to-phase index map
  const PhaseValues<int> cs_phase_dirs{0, 0, 0};     ///< Phase direction signs (±1)

  const int   encoder_direction{0};  ///< Encoder counting direction (+1 or -1)
  const float eangle_offset{0};      ///< Electrical angle offset [rad] at encoder zero

  const float cogging_offset = 0.f;  ///< Shaft angle offset for cogging map lookup [rad]
};

/**
 * @brief Full-featured FOC controller for a single three-phase brushless motor.
 *
 * @details Combines a `BrushlessDriver`, `InlineCurrentSensorPackage`, and an
 * `AbsoluteEncoder` into a closed-loop torque controller. Operates at up to
 * 10 kHz control rate (100 µs period) using a TeensyTimerTool periodic timer
 * or an external tick (when `use_internal_timer = false`).
 *
 * ## Example
 * @code
 * BrushlessController ctrl{U2523, GateDriver2, Current_Sensors2, Encoder1};
 * ctrl.init_components();    // driver init + center-aligned PWM + DMA ADC
 * ctrl.align_sensors();      // encoder direction + electrical angle offset
 * ctrl.set_control_mode(ControllerMode::TORQUE);
 * ctrl.set_target(0.1f);     // 0.1 Nm
 * ctrl.start_control(100, false);
 * // ... in external timer ISR:
 * ctrl.update_sensors();
 * ctrl.update_control();
 * @endcode
 */
template<std::size_t N = 2>
class BrushlessController
{
public:
  BrushlessController() = default;
  ~BrushlessController() = default;

  /**
   * @brief Constructs the controller and computes default PI gains.
   *
   * @details Initial PI gains are set from motor inductance and resistance using
   * bandwidth of 25 rad/s:
   * @f[ K_p = L \cdot 2\pi \cdot 25 \quad[\Omega] \f]
   * @f[ K_i = R \cdot 2\pi \cdot 25 \quad[\Omega/\text{s}] \f]
   *
   * @param motor         Motor electrical parameters (see `motors.hpp`).
   * @param motor_driver  Reference to a `BrushlessDriver` instance.
   * @param current_sensors Reference to an `InlineCurrentSensorPackage`.
   * @param pos_sensor    Reference to an `AbsoluteEncoder` (e.g., `SPIEncoder`).
   */
  BrushlessController(
    MotorParameters motor,
    BrushlessDriver & motor_driver,
    InlineCurrentSensorPackage<N> & current_sensors,
    AbsoluteEncoder & pos_sensor)
  : motor_(motor),
    driver_(motor_driver),
    cs_(current_sensors),
    position_sensor_(pos_sensor),
    ctrl_timer_(TeensyTimerTool::TCK),
    print_timer_(TeensyTimerTool::TCK)
  {
    Kp_ = motor_.phase_L * _2_PI_ * 25.f; // Ohms = V / A
    Ki_ = motor_.phase_R * _2_PI_ * 25.f; // Ohms * s = Vs / A
    set_feedback_control(PIController<QuadDirectValues<float>>(Kp_, Ki_, control_period_s_));
    ff_filter_ = MotorFeedforward(motor_.phase_R, motor_.phase_L, control_period_s_);
    MAX_VOLT_ = 1.5f * motor_.phase_R * motor_.MAX_CURRENT;
    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_current_, filter_cutoff_freq_hz_fb_);
  }

  // -------------------------------------------------------------------------
  // Initialization
  // -------------------------------------------------------------------------

  /**
   * @brief Initialises the driver, sensors, center-aligned PWM, and optionally DMA ADC.
   *
   * @details Performs the full hardware init sequence:
   * 1. `driver_.init()` — configures PWM pins and frequency (edge-aligned).
   * 2. `driver_.configure_center_aligned()` — switches FlexPWM to center-aligned
   *    mode; on success, `driver_.get_trigger_source()` returns the ADC trigger pin.
   * 3. `cs_.init_sensors()` — runs `analogRead()`-based offset calibration. Must
   *    happen **before** the ADC is put in hardware-trigger mode.
   * 4. `configure_adc_trigger()` (if `enable_dma` is `true`) — routes the
   *    FlexPWM trigger through XBAR1 and ADC_ETC to the ADC, then enables DMA
   *    for each sensor in the active controller's package.
   *
   * @param enable_dma If `true` (default), configures center-aligned PWM and the
   *                   FlexPWM → XBAR1 → ADC_ETC → ADC → DMA trigger chain.
   *                   If `false`, the system uses edge-aligned PWM and blocking
   *                   `analogRead()` for all current reads. Useful during
   *                   calibration firmware builds or when the DMA peripheral
   *                   conflicts with another subsystem.
   * @returns `true` if driver and all sensors initialised successfully.
   *
   * @note A warning is printed if center-aligned configuration fails, but
   *       `init_components()` still returns `true` if the driver and sensors
   *       initialised. In that case the system falls back to edge-aligned PWM
   *       and software-polled ADC regardless of `enable_dma`.
   */
  bool init_components(bool enable_dma = true)
  {
    auto ret_d = driver_.init();
    if (!ret_d) { return false; }

    bool ret_ca = false;
    if (enable_dma) {
      // Center-aligned must be configured after init() (establishes VAL1 in FlexPWM)
      ret_ca = driver_.configure_center_aligned();
      if (!ret_ca) {
        nu_log::push(nu_log::Level::WARN, nu_log::Id::PWM_CONFIG_FAIL);
        nu_log::drain(Serial);
      }
    }

    // Sensor offset calibration uses analogRead() — must happen before configure_adc_trigger()
    auto ret_cs = cs_.init_sensors();

    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_current_, filter_cutoff_freq_hz_fb_);

    if (enable_dma && ret_ca && ret_cs) {
      if (!configure_adc_trigger()) {
        nu_log::push(nu_log::Level::WARN, nu_log::Id::ADC_TRIGGER_FAIL);
        nu_log::drain(Serial);
      }
    }

    return ret_d & ret_cs;
  }

  // -------------------------------------------------------------------------
  // RL parameter identification
  // -------------------------------------------------------------------------

  /**
   * @brief Identifies single-phase winding resistance and inductance in situ.
   *
   * @details Performs two sequential blocking measurements using direct phase
   * injection.  No encoder calibration is required: the test works entirely
   * in the physical abc frame and the motor may twitch slightly.
   *
   * ## Measurement sequence
   *
   * ### Step 1 — Resistance (DC step response)
   * Applies a differential DC voltage across phase A relative to phases B and C:
   * @f[ V_A = V_{test} + V_{bias},\quad V_B = V_C = V_{bias} @f]
   * where @f$ V_{bias} = 1\,\text{V} @f$ prevents the PWM counter from
   * reaching zero (which would switch the pin to digital mode).
   *
   * After waiting 20 ms for the current to reach DC steady state (> 5×τ for
   * any motor with L/R < 4 ms), all sensors are sampled `r_avg_samples` times
   * and averaged.  The sensor with the highest absolute reading carries the
   * full phase-A current:
   * @f[ I_A^{ss} = \frac{2}{3}\cdot\frac{V_{test}}{R_{phase}} \f]
   * @f[ R_{phase} = \frac{2}{3}\cdot\frac{V_{test}}{I_{max}} \f]
   *
   * ### Step 2 — Inductance (ODrive-style alternating voltage pulses)
   * Alternates between a positive half-pulse (@f$ V_A @f$ high) and a
   * negative half-pulse (@f$ V_B, V_C @f$ high) of duration `L_pulse_us`.
   * Because the polarity alternates, the DC magnetisation cancels across
   * each pulse pair, preventing rotor lock-up.
   *
   * In steady-state oscillation (reached after a few pulses), the peak-to-peak
   * current swing per cycle — measured on the phase carrying the largest signal —
   * is:
   * @f[ \Delta I = \frac{2\,V_{test}\,t_{pulse}}{L_{eff}},
   *     \quad L_{eff} = \tfrac{3}{2}\,L_{phase} \f]
   * so:
   * @f[ L_{phase} = \frac{2}{3}\cdot\frac{2\,V_{test}\,t_{pulse}}{\Delta I} \f]
   *
   * The identifed sensor is locked in during the R step and reused for all
   * L pulses to ensure consistent sign convention.
   *
   * ### Sanity checks
   * `success` is set to `false` if:
   * - @f$ I_{max} < 0.02\,\text{A} @f$ (sensor not responding — offset error or
   *   motor disconnected)
   * - @f$ R_{phase} \le 0 @f$ or @f$ L_{phase} \le 0 @f$
   * - @f$ \Delta I_{avg} < 0.005\,\text{A} @f$ (pulse too short or test voltage
   *   too low to measure inductance above the noise floor)
   *
   * @param update_params   If `true` (default), writes the identified values back
   *                        to `motor_.phase_R` and `motor_.phase_L` and recomputes
   *                        PI gains and the feedforward filter.
   *                        If `false`, returns values for inspection without changing
   *                        any controller state.
   * @param test_voltage    Differential voltage applied across the phase pair [V].
   *                        Default 1 V.  Must satisfy @f$ V_{test} < V_{bus} - 1 @f$.
   *                        Increase for high-resistance motors (R > 2 Ω); decrease
   *                        for low-resistance motors to stay below `SAFE_CURRENT`.
   * @param n_L_pulses      Number of alternating pulse pairs for the inductance
   *                        estimate.  More pulses reduce noise but extend test time.
   *                        Default 200 (≈ 80 ms at 200 µs per pulse pair).
   * @param L_pulse_us      Duration of each individual half-pulse [µs].  Must
   *                        satisfy @f$ t_{pulse} \ll \tau = L/R @f$ (the RL time
   *                        constant) for the linear approximation to hold.
   *                        Default 200 µs (safe for τ > 0.5 ms).
   * @param r_avg_samples   Number of ADC samples averaged for the DC resistance
   *                        measurement.  Default 128.
   * @returns `RLIdentResult` containing `phase_R` [Ω], `phase_L` [H], and
   *          `success`.  Always populated even when `success == false`.
   *
   * @note Call `init_components()` (which runs `init_sensors()`) before this
   *       function.  The driver is enabled for the duration of the test and
   *       disabled on return.
   *
   * @warning The motor will produce a small torque impulse during the test.
   *          Ensure the load is free to move or clamped before calling.
   *
   * @warning Do not call after `init_dma_sensors()` has been invoked.  The
   *          internal `analogRead()` calls will reset `ADC_SC2` and silently
   *          disable the DMA path.  If DMA is enabled, call `identify_rl()`
   *          during the `init_components()` sequence before DMA setup.
   */
  RLIdentResult identify_rl(bool update_params = true,
                             float test_voltage = 1.f,
                             int   n_L_pulses   = 200,
                             float L_pulse_us   = 200.f,
                             int   r_avg_samples = 128)
  {
    // ------------------------------------------------------------------
    // Setup
    // ------------------------------------------------------------------
    const float V_bias = 1.f;  // Keeps PWM count > 0 on all pins at all times

    driver_.enable();

    // Positive configuration: drive phase A high, B and C at bias
    auto V_pos = PhaseValues<float>{V_bias + test_voltage, V_bias, V_bias};
    // Negative configuration: drive phase A at bias, B and C high
    auto V_neg = PhaseValues<float>{V_bias, V_bias + test_voltage, V_bias + test_voltage};
    // Idle: all phases at bias (zero differential)
    auto V_idle = PhaseValues<float>{V_bias, V_bias, V_bias};

    // ------------------------------------------------------------------
    // Step 1: Resistance — DC steady-state measurement
    // ------------------------------------------------------------------
    driver_.set_phase_voltages(V_pos);
    delay(20);  // wait >5τ; safe upper bound for any motor with τ < 4 ms

    float I_sum[3] = {0.f, 0.f, 0.f};
    for (int s = 0; s < r_avg_samples; ++s) {
      auto raw = cs_.read_unaligned();
      I_sum[0] += raw[0];
      I_sum[1] += raw[1];
      I_sum[2] += raw[2];
      delayMicroseconds(50);
    }

    driver_.set_phase_voltages(V_idle);

    // Average and find which sensor carries the most current (phase A sensor)
    float I_avg[3];
    float I_abs_max = 0.f;
    int   driven_sensor = 0;
    for (int i = 0; i < 3; ++i) {
      I_avg[i] = I_sum[i] / static_cast<float>(r_avg_samples);
      if (fabsf(I_avg[i]) > I_abs_max) {
        I_abs_max   = fabsf(I_avg[i]);
        driven_sensor = i;
      }
    }

    // For Y-connected motor with V_A = V_test, V_B = V_C = 0:
    // I_A = (2/3) * V_test / R_phase  →  R_phase = (2/3) * V_test / I_A
    float R_phase = (2.f / 3.f) * test_voltage / I_abs_max;

    // Let the motor demagnetise before the L test
    delay(5);

    // ------------------------------------------------------------------
    // Step 2: Inductance — ODrive-style alternating voltage pulses
    // ------------------------------------------------------------------
    // Skip the first few pulses while the current oscillation stabilises
    const int skip_pulses = 5;

    float dI_sum    = 0.f;
    int   valid     = 0;

    for (int p = 0; p < n_L_pulses + skip_pulses; ++p) {
      // Positive half-pulse: I_A rises
      driver_.set_phase_voltages(V_pos);
      delayMicroseconds(static_cast<uint32_t>(L_pulse_us));
      float I_peak = cs_.read_unaligned()[driven_sensor];

      // Negative half-pulse: I_A falls
      driver_.set_phase_voltages(V_neg);
      delayMicroseconds(static_cast<uint32_t>(L_pulse_us));
      float I_valley = cs_.read_unaligned()[driven_sensor];

      if (p >= skip_pulses) {
        dI_sum += (I_peak - I_valley);
        ++valid;
      }
    }

    driver_.set_phase_voltages(V_idle);
    driver_.disable();

    // ------------------------------------------------------------------
    // Compute L_phase
    // ------------------------------------------------------------------
    // In steady-state: ΔI = I_peak − I_valley = 2 * V_test * t_pulse / L_eff
    // L_eff = 3/2 * L_phase  (Y-connected, phase A vs B||C)
    // → L_phase = (2/3) * L_eff = (2/3) * (2 * V_test * t_pulse) / ΔI
    float dI_avg  = (valid > 0) ? (dI_sum / static_cast<float>(valid)) : 0.f;
    float t_pulse = L_pulse_us * 1e-6f;
    float L_phase = (valid > 0) ? ((2.f / 3.f) * 2.f * test_voltage * t_pulse / dI_avg) : 0.f;

    // ------------------------------------------------------------------
    // Sanity checks
    // ------------------------------------------------------------------
    bool ok = (I_abs_max  > 0.02f)    // sensor responding
           && (R_phase    > 0.f)
           && (dI_avg     > 0.005f)   // L measurement above noise floor
           && (L_phase    > 0.f);

    RLIdentResult result{R_phase, L_phase, ok};

    nu_log::push(nu_log::Level::INFO, nu_log::Id::RL_RESULT_R, R_phase);
    nu_log::push(nu_log::Level::INFO, nu_log::Id::RL_RESULT_L, L_phase * 1e6f);
    nu_log::push(nu_log::Level::INFO, nu_log::Id::RL_RESULT_DI, dI_avg);
    nu_log::push(nu_log::Level::INFO, nu_log::Id::RL_RESULT_OK, ok ? 1.f : 0.f);
    nu_log::drain(Serial);

    // ------------------------------------------------------------------
    // Apply results to controller state (if requested and valid)
    // ------------------------------------------------------------------
    if (update_params && ok) {
      motor_.phase_R = R_phase;
      motor_.phase_L = L_phase;

      // Recompute default PI gains from new RL values
      Kp_ = motor_.phase_L * _2_PI_ * 25.f;
      Ki_ = motor_.phase_R * _2_PI_ * 25.f;
      set_feedback_control(
        PIController<QuadDirectValues<float>>(Kp_, Ki_, control_period_s_));

      // Rebuild feedforward filter with new R and L
      ff_filter_ = MotorFeedforward(motor_.phase_R, motor_.phase_L, control_period_s_);
    }

    return result;
  }

  // -------------------------------------------------------------------------
  // Calibration
  // -------------------------------------------------------------------------

  /**
   * @brief Runs automated sensor alignment (encoder direction + electrical angle offset).
   *
   * @details Performs a two-pass scan (forward then reverse) in open-loop velocity
   * mode to determine:
   * - `pos_sensor_dir_`: whether the encoder counts in the same direction as the
   *   electrical angle progression (+1 or −1).
   * - `e_ang_offset_`: the electrical angle offset between the encoder zero and
   *   the rotor's d-axis zero. Averaged over both scan passes to reduce error.
   *
   * Also calls `cs_.align_sensors()` first to map sensors to phases.
   *
   * @returns `true` if all sub-steps (sensor align, encoder motion detected,
   *          offset converged) succeeded.
   *
   * @note Rotates the motor through `calibration_scan_distance_` radians
   *       (default π/2) at `calibration_scan_speed_` (default π/4 rad/s).
   *       Ensure the motor is free to rotate before calling.
   *
   * @todo Add explicit `return true` at the end of this function to suppress
   *       `-Wreturn-type` compiler warnings. The function currently falls through
   *       correctly in all paths but lacks a terminal `return`.
   */
  bool align_sensors()
  {
    e_ang_offset_ = 0.f;

    auto ret = cs_.align_sensors(driver_, 0.5f * motor_.phase_R * motor_.SAFE_CURRENT);
    if(!ret){
      nu_log::push(nu_log::Level::ERROR, nu_log::Id::ALIGN_FAIL);
      nu_log::drain(Serial);
      return ret;
    }
    delay(1000);
    // Set inital values
    update_sensors();
    open_loop_shaft_angle_ = 0.f;
    open_loop_shaft_velocity_ = 0.f;

    float init_ang = encoder_angle.get_full_angle();

    float offset_sum = 0.f;
    float offset_pos_sum = 0.f;
    float offset_neg_sum = 0.f;
    int samples = 0;

    nu_log::push(nu_log::Level::INFO, nu_log::Id::ALIGN_SCAN_START, 1.f);
    nu_log::drain(Serial);

    // float inital_shaft_angle = shaft_angle_.get_full_angle();
    set_control_mode(ControllerMode::OPEN_LOOP_VELOCITY);
    target_ = static_cast<float>(calibration_dir_) * calibration_scan_speed_;
    start_control(1000, false);
    while(static_cast<float>(calibration_dir_) * open_loop_shaft_angle_ < calibration_scan_distance_){
        control_step();
        float elec_cmd = get_eangle(open_loop_shaft_angle_);
        float elec_meas = get_eangle(static_cast<float>(calibration_dir_) * pos_sensor_dir_ * encoder_angle.get_full_angle());
        float offset_pos = normalize_angle(elec_cmd - elec_meas);
        float offset_neg = normalize_angle(elec_cmd + elec_meas);
        offset_pos_sum += offset_pos;
        offset_neg_sum += offset_neg;
        samples++;
        delay(1);
    }
    stop_control();

    float new_ang = encoder_angle.get_full_angle();

    if(new_ang > init_ang + 0.1){
      pos_sensor_dir_ = calibration_dir_;
      offset_sum = offset_pos_sum;


    } else {
      if(new_ang < init_ang - 0.1) {
        pos_sensor_dir_ = -calibration_dir_;
        offset_sum = offset_neg_sum;
      }
      else{
        nu_log::push(nu_log::Level::ERROR, nu_log::Id::NO_MOTION_DETECTED, init_ang);
        nu_log::push(nu_log::Level::INFO,  nu_log::Id::ALIGN_FINAL_ANGLE,  new_ang);
        nu_log::drain(Serial);
        return false;
      }
    }
    nu_log::push(nu_log::Level::INFO, nu_log::Id::ENCODER_DIRECTION,
                 static_cast<float>(pos_sensor_dir_));
    nu_log::push(nu_log::Level::INFO, nu_log::Id::ALIGN_SCAN_START, -1.f);
    nu_log::drain(Serial);

    open_loop_shaft_angle_ = 0.f;
    target_ = static_cast<float>(calibration_dir_) * -calibration_scan_speed_;
    start_control(1000, false);
    while(static_cast<float>(calibration_dir_) * open_loop_shaft_angle_ > -calibration_scan_distance_){
        control_step();
        float elec_cmd = get_eangle(open_loop_shaft_angle_);
        float elec_meas = get_eangle(static_cast<float>(calibration_dir_) * pos_sensor_dir_ * encoder_angle.get_full_angle());
        float offset = normalize_angle(elec_cmd - elec_meas);
        offset_sum += offset;
        samples++;
        delay(1);
    }
    stop_control();
    shaft_velocity_ = 0;
    update_sensors();
    stop_control();
    e_ang_offset_ = normalize_angle(offset_sum / samples);

    nu_log::push(nu_log::Level::INFO, nu_log::Id::ZERO_ELEC_ANGLE, e_ang_offset_);
    nu_log::drain(Serial);

    return true;
  }

  /**
   * @brief Loads pre-measured calibration data without running the alignment scan.
   *
   * @param calib_ `BrushlessCalibration` struct with pre-measured values.
   * @returns `true` if calibration was accepted by the sensor package.
   */
  bool load_calibration(BrushlessCalibration calib_)
  {
    auto ret = cs_.load_calibration(calib_.cs_phase_idx, calib_.cs_phase_dirs);

    set_encoder_direction(calib_.encoder_direction);
    set_eangle_offset(calib_.eangle_offset);

    print_calibration();

    return ret;
  }

  /// @brief Pushes sensor calibration values (phase mapping, encoder direction/offset) to the log and drains.
  void print_calibration(){
    cs_.print_calibration();
    nu_log::push(nu_log::Level::INFO, nu_log::Id::ENCODER_DIRECTION,
                 static_cast<float>(pos_sensor_dir_));
    nu_log::push(nu_log::Level::INFO, nu_log::Id::ZERO_ELEC_ANGLE, e_ang_offset_);
    nu_log::drain(Serial);
  }

  // -------------------------------------------------------------------------
  // Control lifecycle
  // -------------------------------------------------------------------------

  /**
   * @brief Sets the controller operating mode.
   * @param ctrl_mode One of `DISABLE`, `OPEN_LOOP_VELOCITY`, or `TORQUE`.
   */
  void set_control_mode(ControllerMode ctrl_mode) { ctrl_mode_ = ctrl_mode; }

  /// @returns Current controller operating mode.
  ControllerMode get_control_mode() const { return ctrl_mode_; }

  /**
   * @brief Configures the control period and optionally starts the internal timer.
   *
   * @details Computes `control_period_s_` and `control_freq_hz_`, rebuilds all
   * filters at the new sampling rate, seeds the state estimators from the current
   * sensor readings, and enables the driver.
   *
   * If `use_internal_timer = true`, a TeensyTimerTool TCK timer is started to
   * call `control_step()` at the configured period. If `false`, the caller is
   * responsible for calling `update_sensors()` and `update_control()` (or
   * `control_step()`) at the correct rate from an external timer callback.
   *
   * @param control_period_us Control loop period in microseconds. 100 µs = 10 kHz.
   * @param use_internal_timer `true` to start the internal TCK timer automatically.
   *
   * @note Call `set_control_mode()` and `set_target()` before calling this function.
   */
  void start_control(int control_period_us, bool use_internal_timer = true)
  {
    // kT = 0 would produce infinite current requests in update_control().
    // Catch the misconfiguration at startup rather than silently at runtime.
    if (motor_.kT <= 0.f) {
      nu_log::push(nu_log::Level::ERROR, nu_log::Id::FAULT_DRIVER_INIT, motor_.kT);
      nu_log::drain(Serial);
      return;
    }

    control_period_us_ = control_period_us;
    control_period_s_ = control_period_us_ * 1e-6;
    control_freq_hz_ = 1.f / control_period_s_;

    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_current_, filter_cutoff_freq_hz_fb_);

    // Reconstruct feedforward filter at the new control period
    ff_filter_ = MotorFeedforward(motor_.phase_R, motor_.phase_L, control_period_s_);

    // Read the encoder via the legacy filter path regardless of PLL mode so that
    // shaft_angle_ is valid before any PLL or filter state is initialised.
    // Using the PLL here would produce a large transient because pos_est_ is still
    // at its default (0 rad) and the error would be the full motor position.
    {
      bool pll_was_active = use_velocity_pll_;
      use_velocity_pll_   = false;
      update_sensors();
      use_velocity_pll_   = pll_was_active;
    }

    if (use_velocity_pll_) {
      velocity_pll_.reset(shaft_angle_, 0.f);
    } else {
      pos_filter_.reset(shaft_angle_);
      vel_filter_.reset(shaft_angle_);
      vel_filter_cutoff_.reset();
    }

    shaft_velocity_ = 0.f;

    last_error_ = QuadDirectValues<float>{0.f, 0.f};
    last_command_ = QuadDirectValues<float>{0.f, 0.f};

    driver_.enable();

    if (use_internal_timer) {
      internal_timer_on_ = true;
      ctrl_timer_.begin(
        [this] {
          control_step();
        }, control_period_us_);
    }
  }

  /**
   * @brief Stops the control loop and disables the driver.
   *
   * @details Stops the internal TCK timer (if running) and calls
   * `driver_.disable()`, which zeroes all phase voltages and pulls the enable
   * pin LOW.
   */
  void stop_control()
  {
    if(internal_timer_on_){
      internal_timer_on_ = false;
      ctrl_timer_.stop();
      }
    driver_.disable();
  }

  /**
   * @brief Starts a periodic Serial.print diagnostic at the given interval.
   * @param print_period_ms Print interval in milliseconds.
   */
  void start_print(int print_period_ms)
  {
    print_timer_.begin(
      [this] {
        printer();
      }, print_period_ms * 1000);
  }

  /// @brief Stops the diagnostic print timer.
  void stop_print() { print_timer_.stop(); }

  // -------------------------------------------------------------------------
  // Main control functions (public for external-timer mode)
  // -------------------------------------------------------------------------

  /**
   * @brief Updates all sensor estimates: position, velocity, phase currents, and thermal model.
   *
   * @details Called once per control step before `update_control()`:
   * 1. Reads encoder → updates `encoder_angle` (multi-turn tracker).
   * 2. Applies position filter → `shaft_angle_` [rad].
   * 3. Computes `electrical_angle_` = `pole_pairs × shaft_angle_ − e_ang_offset_`.
   * 4. Estimates velocity via FIR differentiator + 2nd-order Butterworth.
   * 5. Reads phase currents (DMA or software path, filtered).
   * 6. Applies Park transform → `quaddirect_currents_` (I_q, I_d).
   * 7. If a thermal model is attached and thermal limiting is enabled, advances
   *    the model by one step using the I²R power from the current Park-frame
   *    currents: @f$ P = \frac{1}{2}(I_q^2 + I_d^2) R_{phase} @f$.
   */
  void update_sensors()
  {
    encoder_angle.update_angle(position_sensor_.read());
    if (use_velocity_pll_) {
      velocity_pll_.update(pos_sensor_dir_ * encoder_angle.get_full_angle());
      shaft_angle_    = velocity_pll_.get_position();
      shaft_velocity_ = velocity_pll_.get_velocity();
    } else {
      shaft_angle_    = pos_filter_.update(pos_sensor_dir_ * encoder_angle.get_full_angle());
      shaft_velocity_ = vel_filter_cutoff_.update(vel_filter_.update(shaft_angle_));
    }
    electrical_angle_ = get_eangle(shaft_angle_);
    phase_currents_ = cs_.get_phase_currents(true);
    quaddirect_currents_ = phases_to_quaddirect<float>(phase_currents_, electrical_angle_);

    // Thermal model update — I²R per-phase power from Park-frame currents.
    // Factor of 1/2 converts amplitude-invariant peak current to per-phase RMS power:
    // P_per_phase = (I_q² + I_d²) / 2 * R_phase
    if (thermal_limiting_ && thermal_model_ != nullptr) {
      float I_sq = quaddirect_currents_.q * quaddirect_currents_.q
                 + quaddirect_currents_.d * quaddirect_currents_.d;
      thermal_model_->update(0.5f * I_sq * motor_.phase_R, control_period_s_);
    }
  }

  /**
   * @brief Executes one control step (voltage computation → PWM output).
   *
   * @details Switches on `ctrl_mode_`:
   * - **DISABLE**: returns immediately; no PWM output.
   * - **OPEN_LOOP_VELOCITY**: integrates `target_` (rad/s) to get a simulated
   *   angle, computes the required voltage from resistance and back-EMF estimate,
   *   and applies it in open loop. Used during calibration.
   * - **TORQUE**: Full FOC pipeline (cogging → current request → feedforward +
   *   feedback + BEMF → filter → center + offset → PWM).
   */
  void update_control()
  {
    switch (ctrl_mode_) {
      case ControllerMode::DISABLE:
        return;
      case ControllerMode::OPEN_LOOP_VELOCITY:
        open_loop_shaft_angle_ += (target_ * control_period_s_);
        open_loop_shaft_velocity_ = target_;
        {
          // In this case, the target is a desired shaft velocity
          // Everything is an estimate in open loop

          float back_emf = motor_.kV * target_;

          auto phase_volts = quaddirect_to_phases<float>({motor_.phase_R * motor_.SAFE_CURRENT + back_emf, 0.f},
                                                         get_eangle(open_loop_shaft_angle_));
          auto cntr_volts = center_phase_voltages(phase_volts) + PhaseValues<float>{1.f, 1.f, 1.f};;
          driver_.set_phase_voltages(cntr_volts);
        }
        return;

      case ControllerMode::TORQUE:
        {
          // Add in cogging torque if needed
          if (anticog_enable_) { target_ += torque_mapper_(shaft_angle_ - cogging_offset_); }

          // Convert requested torque into a current request
          float requested_current = target_ / motor_.kT;

          // Hard clamp to motor peak current
          requested_current = std::clamp(requested_current, -motor_.MAX_CURRENT, motor_.MAX_CURRENT);

          // Thermal derating: soft clamp based on estimated winding temperature.
          // Derating begins at 75% of thermal headroom above ambient and scales
          // linearly to zero at T_max_winding.
          if (thermal_limiting_ && thermal_model_ != nullptr) {
            float I_limit = thermal_model_->get_current_limit(motor_.MAX_CURRENT);
            requested_current = std::clamp(requested_current, -I_limit, I_limit);
          }

          // Generate desired current in QD frame, D = 0;
          QuadDirectValues<float> desr_current{requested_current, 0.f};

          PhaseValues<float> ctrl_volts{0.f, 0.f, 0.f};

          // Pump controllers
          if(feedforward_enable_) { ctrl_volts += feedforward(desr_current); }
          if(feedback_enable_) { ctrl_volts += feedback(desr_current); }
          if(back_emf_enable_) { ctrl_volts += back_emf_decoupler(); }

          //Apply filter to these controllers
          auto filtered_ctrl_volts = filter_phase_voltages(ctrl_volts);

          // We don't want to filter this as this is a real thing
          // Although, unless your motor had an insane pole pair count(> 500), the filters should not catch this
          if(anticog_volt_enable_){
            filtered_ctrl_volts += volt_mapper_(shaft_angle_ - cogging_offset_);
          }

          // Shift all voltages by 1 to avoid setting PWM pin to 0 as it will switch to digital and cause delays
          const auto dr_volts = center_phase_voltages(filtered_ctrl_volts) + PhaseValues<float>{1.f, 1.f, 1.f};

          last_phase_volts_ = dr_volts;

          driver_.set_phase_voltages(dr_volts);

        }
        return;

      default:
        return;
    }

  }

  // -------------------------------------------------------------------------
  // Setpoint
  // -------------------------------------------------------------------------

  /**
   * @brief Sets the controller torque (or velocity) target.
   *
   * @details In `TORQUE` mode: `target_` is the desired torque in Nm.
   * In `OPEN_LOOP_VELOCITY` mode: `target_` is the desired shaft velocity in rad/s.
   *
   * @param target Setpoint in Nm (TORQUE) or rad/s (OPEN_LOOP_VELOCITY).
   *
   * @todo Add `__disable_irq()` / `__enable_irq()` guards if `set_target()` is
   *       ever called from a different ISR priority than `update_control()`.
   *       Currently both are invoked from the same TCK timer callback so no data
   *       race exists, but this is fragile if the scheduler changes.
   */
  void set_target(float target) { target_ = target; }

  /// @returns Current control setpoint: Nm (TORQUE) or rad/s (OPEN_LOOP_VELOCITY).
  float get_target() const { return target_; }

  // -------------------------------------------------------------------------
  // State accessors
  // -------------------------------------------------------------------------

  /// @returns Filtered shaft angle (multi-turn, relative to calibration zero) [rad].
  float get_shaft_angle() const { return shaft_angle_; }

  /// @returns Shaft angle wrapped to (−π, π] [rad].
  float get_shaft_radians() const { return normalize_angle(shaft_angle_); }

  /// @returns Raw multi-turn encoder angle (no offset, no direction correction) [rad].
  float get_encoder_angle() const { return encoder_angle.get_full_angle(); }

  /// @returns Raw single-turn encoder angle [0, 2π) [rad].
  float get_encoder_radians() const { return encoder_angle.get_angle(); }

  /// @returns Filtered shaft velocity [rad/s].
  float get_shaft_velocity() const { return shaft_velocity_; }

  /// @returns Current electrical angle [rad].
  float get_electrical_angle() const { return electrical_angle_; }

  /// @returns Most recent measured phase currents (abc frame) [A].
  PhaseValues<float> get_phase_currents() const { return phase_currents_; }

  /// @returns Most recent Park-transformed currents (I_q, I_d) [A].
  QuadDirectValues<float> get_quaddirect_currents() const { return quaddirect_currents_; }

  /// @returns Last applied phase voltages (after centering and offset) [V].
  PhaseValues<float> get_last_phasevolts() const { return last_phase_volts_; }

  /// @returns Motor electrical and mechanical parameters.
  MotorParameters get_motor() const { return motor_; }

  // -------------------------------------------------------------------------
  // Controller configuration
  // -------------------------------------------------------------------------

  /**
   * @brief Updates the voltage and current filter cutoff frequencies.
   *
   * @details Reconstructs all Butterworth 2nd-order filters (applied-voltage,
   * feedback-voltage, and current-sensor filters) at the specified cutoffs.
   * Call after `start_control()` sets the control frequency, or whenever the
   * filter parameters need changing.
   *
   * @param cutoff_freq_hz          Applied voltage filter cutoff [Hz]. Default 2500 Hz.
   * @param filter_cutoff_freq_hz_current Current sensor filter cutoff [Hz]. Default 2500 Hz.
   * @param filter_cutoff_freq_hz_fb Feedback voltage filter cutoff [Hz]. Default 500 Hz.
   */
  void set_filters(
    float cutoff_freq_hz, float filter_cutoff_freq_hz_current,
    float filter_cutoff_freq_hz_fb)
  {
    filter_cutoff_freq_hz_ = cutoff_freq_hz;
    filter_cutoff_freq_hz_current_ = filter_cutoff_freq_hz_current;
    filter_cutoff_freq_hz_fb_ = filter_cutoff_freq_hz_fb;

    cs_.set_filters(Butterworth2nd<float>(filter_cutoff_freq_hz_current_, control_freq_hz_));

    applited_voltage_filters_.a = Butterworth2nd<float>(cutoff_freq_hz, control_freq_hz_);
    applited_voltage_filters_.b = Butterworth2nd<float>(cutoff_freq_hz, control_freq_hz_);
    applited_voltage_filters_.c = Butterworth2nd<float>(cutoff_freq_hz, control_freq_hz_);

    feedback_voltage_filters_.a =
      Butterworth2nd<float>(filter_cutoff_freq_hz_fb_, control_freq_hz_);
    feedback_voltage_filters_.b =
      Butterworth2nd<float>(filter_cutoff_freq_hz_fb_, control_freq_hz_);
    feedback_voltage_filters_.c =
      Butterworth2nd<float>(filter_cutoff_freq_hz_fb_, control_freq_hz_);
  }

  /// @brief Sets the feedback (PI) controller for current control.
  void set_feedback_control(DiscreteFilter<QuadDirectValues<float>, float> fb_filter) { feedback_ = fb_filter; }

  /// @brief Sets the velocity estimator filter.
  void set_velocity_filter(DiscreteFilter<float, float> vel_filter) { vel_filter_ = vel_filter; }

  /// @brief Sets the shaft angle smoothing filter.
  void set_position_filter(DiscreteFilter<float, float> pos_filter) { pos_filter_ = pos_filter; }

  /**
   * @brief Enables the PLL velocity/position estimator, replacing the FIR + Butterworth chain.
   *
   * @param bandwidth_hz Closed-loop −3 dB bandwidth [Hz]. Typical range: 30–500 Hz.
   * @param damping      Damping ratio ζ (default 1/√2 ≈ 0.707 for Butterworth response).
   *
   * @note Rebuilds the PLL at the current `control_period_s_`. If called before
   *       `start_control()`, the PLL will be re-reset by `start_control()` once
   *       the initial encoder reading is available.
   */
  void enable_velocity_pll(float bandwidth_hz, float damping = 0.707f)
  {
    velocity_pll_ = VelocityPLL(bandwidth_hz, damping, control_period_s_);
    velocity_pll_.reset(shaft_angle_, 0.f);
    use_velocity_pll_ = true;
  }

  /// @brief Disables the PLL and reverts to the FIR + Butterworth velocity estimator.
  void disable_velocity_pll() { use_velocity_pll_ = false; }

  /**
   * @brief Sets the proportional gain of the current PI controller.
   *
   * @details Rebuilds the PI controller with the new gain at the current control period.
   * @param kp Proportional gain [Ω = V/A].
   */
  void set_kp(float kp)
  {
    Kp_ = kp;
    set_feedback_control(PIController<QuadDirectValues<float>>(Kp_, Ki_, control_period_s_));
  }

  /// @returns Proportional gain of the current PI controller [Ω].
  float get_kp() const { return Kp_; }

  /**
   * @brief Sets the integral gain of the current PI controller.
   *
   * @details Rebuilds the PI controller with the new gain at the current control period.
   * @param ki Integral gain [Ω/s = Vs/A].
   */
  void set_ki(float ki)
  {
    Ki_ = ki;
    set_feedback_control(PIController<QuadDirectValues<float>>(Kp_, Ki_, control_period_s_));
  }

  /// @returns Integral gain of the current PI controller [Ω/s].
  float get_ki() const { return Ki_; }

  /**
   * @brief Sets the phase voltage ceiling used by `center_phase_voltages()`.
   * @param v Maximum phase voltage [V]. Default: `1.5 * phase_R * MAX_CURRENT`.
   */
  void set_max_voltage(float v) { MAX_VOLT_ = v; }

  /// @returns Phase voltage ceiling [V].
  float get_max_voltage() const { return MAX_VOLT_; }

  // -------------------------------------------------------------------------
  // Feature enables
  // -------------------------------------------------------------------------

  /// @brief Enables or disables the bilinear feedforward voltage term.
  void set_feedforward_state(bool state) { feedforward_enable_ = state; }

  /// @brief Enables or disables the PI feedback current controller.
  void set_feedback_state(bool state) { feedback_enable_ = state; }

  /// @brief Enables or disables the back-EMF decoupling voltage.
  void set_back_emf_comp_state(bool state) { back_emf_enable_ = state; }

  /// @brief Enables verbose Serial debug output.
  void enable_debug_print() { debug_print_ = true; }

  /// @brief Suppresses verbose Serial debug output.
  void disable_debug_print() { debug_print_ = false; }

  // -------------------------------------------------------------------------
  // Calibration configuration
  // -------------------------------------------------------------------------

  /**
   * @brief Sets the encoder counting direction relative to the motor's convention.
   * @param dir +1 if encoder counts in the same direction as motor rotation, −1 otherwise.
   */
  void set_encoder_direction(int dir) {
    if(dir == 1){ pos_sensor_dir_= 1; return;}
    if(dir == -1){ pos_sensor_dir_= -1; return;}
    else{
      nu_log::push(nu_log::Level::WARN, nu_log::Id::INVALID_DIRECTION,
                   static_cast<float>(dir));
      nu_log::drain(Serial);
      return;
    }
  }

  /// @returns Electrical angle offset from `align_sensors()` [rad].
  float get_eangle_offset() const { return e_ang_offset_; }

  /// @brief Sets the electrical angle offset between encoder zero and d-axis zero [rad].
  void set_eangle_offset(float offset) { e_ang_offset_ = offset; }

  /// @brief Sets calibration scan angular speed [rad/s].
  void set_calibration_scan_speed(float w) { calibration_scan_speed_ = w; }

  /// @brief Sets calibration scan distance [rad].
  void set_calibration_scan_range(float rads) { calibration_scan_distance_ = rads; }

  /**
   * @brief Sets the calibration scan direction (+1 or −1).
   * @param dir +1 = positive rotation, −1 = negative rotation during alignment.
   */
  void set_calibration_direction(int dir) {
      if(dir == 1){ calibration_dir_= 1; return;}
      if(dir == -1){ calibration_dir_= -1; return;}
      else{
        Serial.println("Error: Invalid Direction. Please use 1 or -1");
        return;
      }
  }

  // -------------------------------------------------------------------------
  // Anticogging
  // -------------------------------------------------------------------------

  /**
   * @brief Enables torque-based anticogging compensation.
   *
   * @details On each control step, `torque_mapper_(shaft_angle_)` is added to
   * the torque target before current conversion. The mapper is typically built
   * from a `CoggingMapper` measurement run.
   *
   * @param torque_mapper `std::function<float(float)>` from shaft angle [rad]
   *                      to compensation torque [Nm].
   */
  void enable_anticog(const std::function<float(float)> & torque_mapper)
  {
    disable_anticog();
    anticog_enable_ = true;
    torque_mapper_ = torque_mapper;
  }

  /**
   * @brief Enables voltage-based anticogging compensation.
   *
   * @details Adds `volt_mapper_(shaft_angle_)` (three-phase voltages) directly
   * to the phase voltages after filtering, bypassing the current controller.
   * Used when the cogging disturbance is best characterised in voltage space.
   *
   * @param volt_mapper `std::function<PhaseValues<float>(float)>` from shaft
   *                    angle [rad] to three-phase compensation voltages [V].
   */
  void enable_anticog(const std::function<PhaseValues<float>(float)> & volt_mapper)
  {
    disable_anticog();
    anticog_volt_enable_ = true;
    volt_mapper_ = volt_mapper;
  }

  /// @brief Disables all anticogging compensation and resets mappers to zero.
  void disable_anticog()
  {
    anticog_volt_enable_ = false;
    anticog_enable_ = false;
    torque_mapper_ = [](float angle) -> float {return 0.f;};
    volt_mapper_ = [](float angle) -> PhaseValues<float> {return {0.f, 0.f, 0.f};};
  }

  /// @brief Sets the mechanical angle offset for cogging map lookup [rad].
  void set_cogging_offset(float offset) { cogging_offset_ = offset; }

  // -------------------------------------------------------------------------
  // Thermal limiting
  // -------------------------------------------------------------------------

  /**
   * @brief Attaches a `TwoNodeThermalModel` and enables current derating.
   *
   * @details The model is updated once per control tick inside `update_sensors()`
   * using the I²R power computed from the Park-frame currents.  Its
   * `get_current_limit()` result replaces the fixed `motor_.MAX_CURRENT` clamp
   * in `update_control()`.  The model must have been constructed with the correct
   * thermal parameters from `motor_` before being attached here.
   *
   * @param model Non-owning pointer to a `TwoNodeThermalModel`.  Must outlive
   *              the controller.  Pass `nullptr` to detach the model and revert
   *              to fixed current limiting.
   *
   * @note Call `model.reset()` with the expected ambient temperature before
   *       calling `start_control()`, so the initial winding temperature state
   *       is correct.
   *
   * @note If `identify_rl()` is called after attaching the model, the model's
   *       thermal parameters are not updated automatically — call
   *       `model.set_R_th_winding_case()` etc. separately if needed.
   */
  void set_thermal_model(TwoNodeThermalModel * model)
  {
    thermal_model_ = model;
    thermal_limiting_ = (model != nullptr);
  }

  /**
   * @brief Enables current derating via the attached thermal model.
   *
   * @details Has no effect if no model has been attached via `set_thermal_model()`.
   */
  void enable_thermal_limiting()
  {
    if (thermal_model_ != nullptr) { thermal_limiting_ = true; }
  }

  /// @brief Suspends thermal current derating without detaching the model.
  void disable_thermal_limiting() { thermal_limiting_ = false; }

  /// @returns `true` if thermal derating is currently active.
  bool is_thermal_limiting_enabled() const { return thermal_limiting_; }

  /**
   * @returns Pointer to the attached `TwoNodeThermalModel`, or `nullptr` if
   *          none has been set.
   */
  TwoNodeThermalModel * get_thermal_model() const { return thermal_model_; }

private:
  MotorParameters              motor_;
  BrushlessDriver &            driver_;
  InlineCurrentSensorPackage<N> & cs_;
  AbsoluteEncoder &            position_sensor_;

  PhaseValues<Butterworth2nd<float>> applited_voltage_filters_; ///< Applied-voltage smoothing
  PhaseValues<Butterworth2nd<float>> feedback_voltage_filters_;  ///< Feedback-voltage smoothing

  float filter_cutoff_freq_hz_         = 2500.f; ///< Applied voltage filter cutoff [Hz]
  float filter_cutoff_freq_hz_current_ = 2500.f; ///< Current sensor filter cutoff [Hz]
  float filter_cutoff_freq_hz_vel_     = 100.f;  ///< Velocity estimator cutoff [Hz] (unused directly)
  float filter_cutoff_freq_hz_fb_      = 500.f;  ///< Feedback voltage filter cutoff [Hz]

  float calibration_scan_speed_    = 0.25f * PI; ///< Open-loop calibration scan speed [rad/s]
  float calibration_scan_distance_ = 0.5f * PI;  ///< Calibration scan angular range [rad]
  int   calibration_dir_           = 1;           ///< Calibration scan direction (+1 or -1)

  // Position / state variables
  Angle encoder_angle{0, 0.f};          ///< Multi-turn encoder angle tracker
  int   pos_sensor_dir_ = 1;            ///< Encoder direction relative to motor (+1/-1)
  float shaft_angle_    = 0.f;          ///< Filtered shaft angle [rad]
  float shaft_velocity_ = 0.f;          ///< Estimated shaft velocity [rad/s]
  float e_ang_offset_   = 0.f;          ///< Electrical angle zero offset [rad]
  float electrical_angle_ = 0.f;        ///< Current electrical angle [rad]

  DiscreteFilter<float, float> pos_filter_{{1.f}, {}};               ///< 1-tap position smoother (FIR)
  DiscreteFilter<float, float> vel_filter_{{10000.f, -10000.f}, {}}; ///< 2-tap differentiating FIR
  Butterworth2nd<float> vel_filter_cutoff_ = Butterworth2nd<float>(100.f, 10000.f); ///< 2nd-order 100 Hz LP

  VelocityPLL velocity_pll_;           ///< PLL position/velocity observer
  bool         use_velocity_pll_ = true; ///< Route sensor update through PLL instead of FIR+Butterworth

  float cogging_offset_ = 0.f;  ///< Shaft angle offset for cogging map lookup [rad]

  float target_; ///< Control setpoint: Nm (TORQUE) or rad/s (OPEN_LOOP_VELOCITY)

  // Open-loop calibration state
  float open_loop_shaft_angle_    = 0.f; ///< Integrated open-loop angle estimate [rad]
  float open_loop_shaft_velocity_ = 0.f; ///< Open-loop velocity setpoint [rad/s]

  PhaseValues<float>      phase_currents_{0.f, 0.f, 0.f};       ///< Measured phase currents [A]
  QuadDirectValues<float> quaddirect_currents_{0.f, 0.f};        ///< Park-transformed currents [A]

  bool feedforward_enable_ = true;  ///< Enable bilinear RL feedforward
  bool feedback_enable_    = true;  ///< Enable PI current feedback
  bool back_emf_enable_    = true;  ///< Enable back-EMF decoupling

  // Thermal model (non-owning pointer; nullptr = no thermal limiting)
  TwoNodeThermalModel * thermal_model_  = nullptr; ///< Attached two-node thermal model
  bool                  thermal_limiting_ = false;  ///< `true` when derating is active

  bool internal_timer_on_ = false;

  // PI gains
  float Kp_; ///< Proportional gain [Ω = V/A]
  float Ki_; ///< Integral gain [Ω/s = Vs/A]

  QuadDirectValues<float> last_error_{0.f, 0.f};    ///< Previous PI error (for integrator)
  QuadDirectValues<float> last_command_{0.f, 0.f};  ///< Previous PI output (unused directly)

  DiscreteFilter<QuadDirectValues<float>, float> feedback_; ///< PI controller

  /**
   * @brief Bilinear RL feedforward filter (abc frame, per-phase).
   *
   * @details Implements @f$ V[n] = b_0 I[n] + b_1 I[n-1] - V[n-1] @f$ using
   * `MotorFeedforward`, which extends `DiscreteFilter<PhaseValues<float>, float>`.
   * Replaces the former manual `last_desr_phase_currents_` / `last_desr_phase_voltages_`
   * state variables. Reconstructed in the constructor and in `start_control()`
   * whenever the control period changes, which also resets the filter state.
   */
  MotorFeedforward ff_filter_;

  PhaseValues<float> last_phase_volts_{0.f, 0.f, 0.f}; ///< Last applied voltages (for cogging mapper)

  ControllerMode ctrl_mode_ = ControllerMode::DISABLE;

  TeensyTimerTool::PeriodicTimer ctrl_timer_;  ///< Internal control loop timer
  TeensyTimerTool::PeriodicTimer print_timer_; ///< Diagnostic print timer

  int   control_period_us_ = 100;          ///< Control period [µs]
  float control_period_s_  = 100.f * 1e-6f; ///< Control period [s]
  float control_freq_hz_   = 10000.f;      ///< Control frequency [Hz]

  float MAX_VOLT_ = 3.f; ///< Voltage ceiling for center_phase_voltages() [V]

  bool anticog_enable_      = false; ///< Torque anticogging active
  bool anticog_volt_enable_ = false; ///< Voltage anticogging active
  std::function<float(float)>               torque_mapper_ = [](float) -> float            { return 0.f; };
  std::function<PhaseValues<float>(float)>  volt_mapper_   = [](float) -> PhaseValues<float>{ return {0.f,0.f,0.f}; };

  bool debug_print_ = true;

  template <typename T>
  void debug_print(T msg)
  {
    if (!debug_print_) {return;}
    Serial.print(msg);
  }

  template <typename T>
  void debug_println(T msg)
  {
    if (!debug_print_) {return;}
    Serial.println(msg);
  }

  /**
   * @brief One complete control step: read sensors then compute and apply voltage.
   * @details Called by the internal TCK timer when `use_internal_timer = true`.
   */
  void control_step()
  {
    update_sensors();
    update_control();
  }

  /**
   * @brief Configures the FlexPWM → XBAR1 → ADC_ETC → ADC → DMA chain for all
   *        sensors in the active controller's current-sensor package.
   *
   * @details Called from `init_components()` after `driver_.configure_center_aligned()`
   * has set up the FlexPWM trigger output on phase A's submodule.
   *
   * All XBAR routing, ADC_ETC programming, ADC hardware-trigger mode, and DMA
   * channel setup is delegated to `cs_.init_dma_sensors()`. The trigger source
   * (FlexPWMPin identifying the submodule and channel generating the counter-TOP
   * pulse) is passed from the driver.
   *
   * @returns `true` if all sensors in the package were configured successfully.
   *          `false` if the driver has no valid trigger source or any sensor's
   *          pin does not map to an ADC channel.
   *
   * @warning Must be called **after** `cs_.init_sensors()` (blocking `analogRead()`
   *          offset calibration). Calling in the wrong order will reset `ADC_SC2`
   *          and silently disable the DMA trigger path.
   */
  bool configure_adc_trigger()
  {
    auto trigger_src = driver_.get_trigger_source();
    if (trigger_src.module == nullptr) { return false; }

    // All XBAR routing, ADC_ETC, ADC hardware-trigger, and DMA setup is
    // performed inside init_dma_sensors() on a per-sensor basis.
    return cs_.init_dma_sensors(trigger_src);
  }

  /**
   * @brief Bilinear (Tustin) feedforward voltage from desired current trajectory,
   *        implemented via the `MotorFeedforward` discrete filter.
   *
   * @details Converts the desired dq-frame current to the abc phase frame, then
   * passes it through `ff_filter_` (a `MotorFeedforward` instance), which
   * implements:
   * @f[
   *   V[n] = \underbrace{\left(\frac{2L}{T}+R\right)}_{b_0} I_{\text{des}}[n]
   *         + \underbrace{\left(-\frac{2L}{T}+R\right)}_{b_1} I_{\text{des}}[n-1]
   *         - V[n-1]
   * @f]
   * using the `DiscreteFilter` difference equation with coefficients:
   * - @f$ b_0 = 2L/T + R @f$, @f$ b_1 = -2L/T + R @f$ (feedforward)
   * - @f$ a_0 = 1 @f$ (IIR feedback — one-sample voltage memory)
   *
   * The filter state (I[n-1] and V[n-1]) is maintained internally by `ff_filter_`
   * and is reset by `MotorFeedforward::reset()` if needed. The filter is
   * reconstructed (and state cleared) whenever `start_control()` is called
   * with a new control period.
   *
   * @param desr_current Desired q- and d-axis current [A] in the rotating frame.
   * @returns Feedforward phase voltage command in the ABC frame [V].
   */
  PhaseValues<float> feedforward(QuadDirectValues<float> desr_current)
  {
    // Convert desired dq current to abc phase frame, then apply RL inversion filter
    auto desr_phase_currents = quaddirect_to_phases<float>(desr_current, electrical_angle_);
    return ff_filter_.update(desr_phase_currents);
  }

  /**
   * @brief PI feedback voltage from measured vs. desired current error.
   *
   * @details Computes @f$ e = I_{\text{des}} - I_{\text{meas}} @f$ in the
   * rotating (q/d) frame, passes it through the `PIController`, transforms the
   * result back to the ABC frame, and applies the feedback voltage filter.
   *
   * PI gains default to:
   * @f[ K_p = L \cdot 2\pi \cdot 25, \quad K_i = R \cdot 2\pi \cdot 25 @f]
   *
   * @param desr_current Desired q/d current [A].
   * @returns Feedback phase voltage command in ABC frame [V], filtered at
   *          `filter_cutoff_freq_hz_fb_`.
   */
  PhaseValues<float> feedback(QuadDirectValues<float> desr_current)
  {
    QuadDirectValues<float> error = desr_current - quaddirect_currents_;

    QuadDirectValues<float> pi_out = feedback_.update(error);

    // Anti-windup: clamp PI output to the phase voltage ceiling and back-calculate
    // the integrator state so it doesn't accumulate beyond what the driver can apply.
    // This prevents integrator windup during current saturation (e.g. high speed,
    // mechanical lock, or thermal derating).
    QuadDirectValues<float> pi_clamped = {
      std::clamp(pi_out.q, -MAX_VOLT_, MAX_VOLT_),
      std::clamp(pi_out.d, -MAX_VOLT_, MAX_VOLT_)
    };
    if (pi_out.q != pi_clamped.q || pi_out.d != pi_clamped.d) {
      feedback_.set_output_state(pi_clamped);
    }

    auto fb_phs_v = quaddirect_to_phases<float>(pi_clamped, electrical_angle_);

    return filter_feedback_voltages(fb_phs_v);
  }

  /**
   * @brief Back-EMF decoupling voltage in the rotating frame.
   *
   * @details Estimates the back-EMF as:
   * @f[ V_{\text{BEMF}} = k_V \cdot \omega @f]
   * where @f$ \omega @f$ is the measured shaft velocity. Adds this as a q-axis
   * feedforward voltage to decouple the current controller from motor speed.
   *
   * @returns BEMF compensation voltage in ABC frame [V], clamped to
   *          @f$ \pm k_V \cdot 300 @f$ V (300 rad/s ≈ maximum no-load speed).
   *
   * @note `kV` for the U2523/U2535 is very small (0.001 V·s/rad), so the BEMF
   *       contribution is negligible at typical operating speeds. It is included
   *       for completeness and correctness at high speeds.
   */
  PhaseValues<float> back_emf_decoupler()
  {
    auto bemf_volt = std::clamp(
      motor_.kV * shaft_velocity_, -motor_.kV * 300.f,
      motor_.kV * 300.f);

    auto bemf =
      quaddirect_to_phases<float>({bemf_volt, 0.f}, electrical_angle_);
    return bemf;
  }

  /// @brief Prints shaft velocity to Serial (called by print_timer_).
  void printer()
  {
    Serial.println(shaft_velocity_);
  }

  /**
   * @brief Converts mechanical shaft angle to electrical angle.
   *
   * @param mech_ang Shaft angle in radians (any range).
   * @returns Electrical angle: @f$ \theta_e = (p \cdot \theta_m) - \delta @f$
   *          normalised to (−π, π], where @f$ p @f$ is the pole-pair count and
   *          @f$ \delta @f$ is `e_ang_offset_`.
   */
  float get_eangle(float mech_ang) const
  {
    return normalize_angle(static_cast<float>(motor_.pole_pairs) * (mech_ang) - e_ang_offset_);
  }

  /**
   * @brief Shifts and scales three-phase voltages to maximise modulation range.
   *
   * @details Implements Space-Vector-PWM-like common-mode injection:
   * 1. Subtracts the minimum phase voltage from all three phases (shifts the
   *    minimum to zero, maximising positive headroom).
   * 2. If the resulting maximum exceeds `MAX_VOLT_`, scales all three phases
   *    down by `MAX_VOLT_ / max` to stay within the voltage ceiling.
   *
   * This doubles the effective modulation range compared to sinusoidal PWM
   * without adding triplen harmonics (they cancel in a star-connected load).
   *
   * @param phase_volts Raw output phase voltages [V].
   * @returns Centred and bounded phase voltages in [0, MAX_VOLT_] [V].
   */
  PhaseValues<float> center_phase_voltages(PhaseValues<float> phase_volts) const
  {
    float _min = min(phase_volts.a, min(phase_volts.b, phase_volts.c));

    PhaseValues<float> offset_volts{_min, _min, _min};

    auto new_volts = phase_volts - offset_volts;

    float _max = max(new_volts.a, max(new_volts.b, new_volts.c));
    float ratio = 1.f;
    if (_max > MAX_VOLT_) {
      ratio = MAX_VOLT_ / _max;
    }
    return new_volts * ratio;
  }

  /// @brief Applies the applied-voltage Butterworth filter to all three phases.
  PhaseValues<float> filter_phase_voltages(PhaseValues<float> phase_volts)
  {
    return {applited_voltage_filters_.a.update(phase_volts.a),
            applited_voltage_filters_.b.update(phase_volts.b),
            applited_voltage_filters_.c.update(phase_volts.c)};
  }

  /// @brief Applies the feedback-voltage Butterworth filter to all three phases.
  PhaseValues<float> filter_feedback_voltages(PhaseValues<float> phase_volts)
  {
    return {feedback_voltage_filters_.a.update(phase_volts.a),
            feedback_voltage_filters_.b.update(phase_volts.b),
            feedback_voltage_filters_.c.update(phase_volts.c)};
  }

};

#endif // BRUSHLESS_CONTROLLER_HPP
