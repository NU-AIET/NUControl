#ifndef NUCONTROL_CORE_BRUSHLESS_CONTROLLER_HPP
#define NUCONTROL_CORE_BRUSHLESS_CONTROLLER_HPP
#include <functional>
#include <cmath>
#include <algorithm>
#include <string>
#include "transformations.hpp"
#include "encoder_interface.hpp"
#include "driver_interface.hpp"
#include "current_sensor_interface.hpp"
#include "discrete_filter.hpp"
#include "motors.hpp"

enum ControllerMode
{
  DISABLE,
  OPEN_LOOP_VELOCITY,
  TORQUE,
};

struct BrushlessCalibration
{
  const PhaseValues<int> cs_phase_idx{-1, -1, -1};
  const PhaseValues<int> cs_phase_dirs{0, 0, 0};

  const int encoder_direction{0};
  const float eangle_offset{0};

  const float cogging_offset = 0.f;
};

class BrushlessController
{
public:
  using LogFn   = std::function<void(const std::string &)>;
  using SleepFn = std::function<void(int /*ms*/)>;

  BrushlessController() = default;
  ~BrushlessController() = default;

  BrushlessController(
    MotorParameters motor,
    IBrushlessDriver & motor_driver,
    ICurrentSensorPackage & current_sensors,
    IAbsoluteEncoder & pos_sensor,
    SleepFn sleep_fn,
    LogFn   log_fn   = [](const std::string &){})
  : motor_(motor),
    driver_(motor_driver),
    cs_(current_sensors),
    position_sensor_(pos_sensor),
    sleep_(sleep_fn),
    log_(log_fn)
  {
    Kp_ = motor_.phase_L * _2_PI_ * 25.f;
    Ki_ = motor_.phase_R * _2_PI_ * 25.f;
    set_feedback_control(PIController<QuadDirectValues<float>>(Kp_, Ki_, control_period_s_));
    MAX_VOLT_ = 1.5f * motor_.phase_R * motor_.MAX_CURRENT;
    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_current_, filter_cutoff_freq_hz_fb_);
  }

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

    feedback_voltage_filters_.a = Butterworth2nd<float>(filter_cutoff_freq_hz_fb_, control_freq_hz_);
    feedback_voltage_filters_.b = Butterworth2nd<float>(filter_cutoff_freq_hz_fb_, control_freq_hz_);
    feedback_voltage_filters_.c = Butterworth2nd<float>(filter_cutoff_freq_hz_fb_, control_freq_hz_);
  }

  void set_control_mode(ControllerMode ctrl_mode) { ctrl_mode_ = ctrl_mode; }

  void start_control(int control_period_us)
  {
    control_period_us_ = control_period_us;
    control_period_s_ = control_period_us_ * 1e-6f;
    control_freq_hz_ = 1.f / control_period_s_;

    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_current_, filter_cutoff_freq_hz_fb_);

    encoder_angle.update_angle(position_sensor_.read());
    float raw_angle = pos_sensor_dir_ * encoder_angle.get_full_angle();

    pos_filter_.reset(raw_angle);
    vel_filter_.reset(raw_angle);
    vel_filter_cutoff_.reset();

    update_sensors();

    shaft_velocity_ = 0.f;

    last_error_   = QuadDirectValues<float>{0.f, 0.f};
    last_command_ = QuadDirectValues<float>{0.f, 0.f};

    last_desr_phase_currents_ = PhaseValues<float>{0.f, 0.f, 0.f};
    last_desr_phase_voltages_ = PhaseValues<float>{0.f, 0.f, 0.f};
    driver_.enable();
  }

  void stop_control() { driver_.disable(); }

  void set_feedback_control(DiscreteFilter<QuadDirectValues<float>, float> fb_filter) { feedback_ = fb_filter; }

  void set_velocity_filter(DiscreteFilter<float, float> vel_filter) { vel_filter_ = vel_filter; }

  void set_position_filter(DiscreteFilter<float, float> pos_filter) { pos_filter_ = pos_filter; }

  bool init_components()
  {
    auto ret_d  = driver_.init();
    auto ret_cs = cs_.init_sensors();
    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_current_, filter_cutoff_freq_hz_fb_);
    return ret_d & ret_cs;
  }

  void print_calibration()
  {
    log_("=====");
    cs_.print_calibration();
    log_("Encoder Direction: " + std::to_string(pos_sensor_dir_));
    log_("Encoder Offset: "    + std::to_string(e_ang_offset_));
    log_("=====");
  }

  bool align_sensors()
  {
    e_ang_offset_ = 0.f;

    auto ret = cs_.align_sensors(driver_, 0.5f * motor_.phase_R * motor_.SAFE_CURRENT);
    if (!ret) {
      log_("Drivers failed to align");
      return ret;
    }
    sleep_(1000);

    update_sensors();
    open_loop_shaft_angle_    = 0.f;
    open_loop_shaft_velocity_ = 0.f;

    float init_ang = encoder_angle.get_full_angle();

    float offset_sum     = 0.f;
    float offset_pos_sum = 0.f;
    float offset_neg_sum = 0.f;
    int samples = 0;

    log_("Forward Move");

    set_control_mode(ControllerMode::OPEN_LOOP_VELOCITY);
    target_ = static_cast<float>(calibration_dir_) * calibration_scan_speed_;
    start_control(1000);
    while (static_cast<float>(calibration_dir_) * open_loop_shaft_angle_ < calibration_scan_distance_) {
      control_step();
      float elec_cmd = get_eangle(open_loop_shaft_angle_);
      float elec_meas = get_eangle(
        static_cast<float>(calibration_dir_) * pos_sensor_dir_ * encoder_angle.get_full_angle());
      offset_pos_sum += normalize_angle(elec_cmd - elec_meas);
      offset_neg_sum += normalize_angle(elec_cmd + elec_meas);
      samples++;
      sleep_(1);
    }
    stop_control();

    float new_ang = encoder_angle.get_full_angle();

    if (new_ang > init_ang + 0.1f) {
      pos_sensor_dir_ = calibration_dir_;
      offset_sum = offset_pos_sum;
    } else if (new_ang < init_ang - 0.1f) {
      pos_sensor_dir_ = -calibration_dir_;
      offset_sum = offset_neg_sum;
    } else {
      log_("No motion detected. Is Encoder Working?");
      log_("Initial Angle: " + std::to_string(init_ang));
      log_("Final Angle: "   + std::to_string(new_ang));
      return false;
    }

    log_("Direction: " + std::to_string(pos_sensor_dir_));
    log_("Reverse Move");

    open_loop_shaft_angle_ = 0.f;
    target_ = static_cast<float>(calibration_dir_) * -calibration_scan_speed_;
    start_control(1000);
    while (static_cast<float>(calibration_dir_) * open_loop_shaft_angle_ > -calibration_scan_distance_) {
      control_step();
      float elec_cmd  = get_eangle(open_loop_shaft_angle_);
      float elec_meas = get_eangle(
        static_cast<float>(calibration_dir_) * pos_sensor_dir_ * encoder_angle.get_full_angle());
      offset_sum += normalize_angle(elec_cmd - elec_meas);
      samples++;
      sleep_(1);
    }
    stop_control();
    shaft_velocity_ = 0.f;
    update_sensors();
    stop_control();
    e_ang_offset_ = normalize_angle(offset_sum / samples);

    log_("Zero Electrical Angle: " + std::to_string(e_ang_offset_));
    return ret;
  }

  bool load_calibration(BrushlessCalibration calib_)
  {
    auto ret = cs_.load_calibration(calib_.cs_phase_idx, calib_.cs_phase_dirs);
    set_encoder_direction(calib_.encoder_direction);
    set_eangle_offset(calib_.eangle_offset);
    print_calibration();
    return ret;
  }

  float get_eangle_offset()  const { return e_ang_offset_; }
  float get_shaft_angle()    const { return shaft_angle_; }
  float get_shaft_radians()  const { return normalize_angle(shaft_angle_); }
  float get_encoder_angle()  const { return encoder_angle.get_full_angle(); }
  float get_encoder_radians()const { return encoder_angle.get_angle(); }
  float get_shaft_velocity() const { return shaft_velocity_; }
  MotorParameters get_motor()const { return motor_; }

  void set_encoder_direction(int dir)
  {
    if (dir == 1)       { pos_sensor_dir_ = 1;  return; }
    if (dir == -1)      { pos_sensor_dir_ = -1; return; }
    log_("Error: Invalid Direction. Please use 1 or -1");
  }

  void set_eangle_offset(float offset)          { e_ang_offset_ = offset; }
  void set_calibration_scan_speed(float w)      { calibration_scan_speed_ = w; }
  void set_calibration_scan_range(float rads)   { calibration_scan_distance_ = rads; }

  void set_calibation_direction(int dir)
  {
    if (dir == 1)  { calibration_dir_ = 1;  return; }
    if (dir == -1) { calibration_dir_ = -1; return; }
    log_("Error: Invalid Direction. Please use 1 or -1");
  }

  void enable_anticog(const std::function<float(float)> & torque_mapper)
  {
    disable_anticog();
    anticog_enable_ = true;
    torque_mapper_  = torque_mapper;
  }

  void enable_anticog(const std::function<PhaseValues<float>(float)> & volt_mapper)
  {
    disable_anticog();
    anticog_volt_enable_ = true;
    volt_mapper_         = volt_mapper;
  }

  void disable_anticog()
  {
    anticog_volt_enable_ = false;
    anticog_enable_      = false;
    torque_mapper_ = [](float) -> float { return 0.f; };
    volt_mapper_   = [](float) -> PhaseValues<float> { return {0.f, 0.f, 0.f}; };
  }

  void set_feedforward_state(bool state)    { feedforward_enable_ = state; }
  void set_feedback_state(bool state)       { feedback_enable_    = state; }
  void set_back_emf_comp_state(bool state)  { back_emf_enable_    = state; }
  void set_cogging_offset(float offset)     { cogging_offset_     = offset; }
  void set_target(float target)             { target_             = target; }

  float get_target() const { return target_; }

  PhaseValues<float>      get_last_phasevolts()       const { return last_phase_volts_; }
  PhaseValues<float>      get_phase_currents()        const { return phase_currents_; }
  QuadDirectValues<float> get_quaddirect_currents()   const { return quaddirect_currents_; }

  void update_sensors()
  {
    encoder_angle.update_angle(position_sensor_.read());
    shaft_angle_      = pos_filter_.update(pos_sensor_dir_ * encoder_angle.get_full_angle());
    electrical_angle_ = get_eangle(shaft_angle_);
    auto raw_vel      = vel_filter_.update(shaft_angle_);
    shaft_velocity_   = vel_filter_cutoff_.update(raw_vel);
    phase_currents_        = cs_.get_phase_currents(true);
    quaddirect_currents_   = phases_to_quaddirect<float>(phase_currents_, electrical_angle_);
  }

  void update_control()
  {
    switch (ctrl_mode_) {
      case ControllerMode::DISABLE:
        return;

      case ControllerMode::OPEN_LOOP_VELOCITY:
        open_loop_shaft_angle_    += (target_ * control_period_s_);
        open_loop_shaft_velocity_  = target_;
        {
          float back_emf  = motor_.kV * target_;
          auto phase_volts = quaddirect_to_phases<float>(
            {motor_.phase_R * motor_.SAFE_CURRENT + back_emf, 0.f},
            get_eangle(open_loop_shaft_angle_));
          auto cntr_volts = center_phase_voltages(phase_volts) + PhaseValues<float>{1.f, 1.f, 1.f};
          driver_.set_phase_voltages(cntr_volts);
        }
        return;

      case ControllerMode::TORQUE:
        {
          if (anticog_enable_) { target_ += torque_mapper_(shaft_angle_ - cogging_offset_); }

          float requested_current = target_ / motor_.kT;
          requested_current = std::clamp(requested_current, -motor_.MAX_CURRENT, motor_.MAX_CURRENT);

          QuadDirectValues<float> desr_current{requested_current, 0.f};

          PhaseValues<float> ctrl_volts{0.f, 0.f, 0.f};
          if (feedforward_enable_) { ctrl_volts += feedforward(desr_current); }
          if (feedback_enable_)    { ctrl_volts += feedback(desr_current); }
          if (back_emf_enable_)    { ctrl_volts += back_emf_decoupler(); }

          auto filtered_ctrl_volts = filter_phase_voltages(ctrl_volts);

          if (anticog_volt_enable_) {
            filtered_ctrl_volts += volt_mapper_(shaft_angle_ - cogging_offset_);
          }

          const auto dr_volts = center_phase_voltages(filtered_ctrl_volts) + PhaseValues<float>{1.f, 1.f, 1.f};
          last_phase_volts_ = dr_volts;
          driver_.set_phase_voltages(dr_volts);
        }
        return;

      default:
        return;
    }
  }

  void control_step()
  {
    update_sensors();
    update_control();
  }

private:
  MotorParameters motor_;
  IBrushlessDriver & driver_;
  ICurrentSensorPackage & cs_;
  IAbsoluteEncoder & position_sensor_;

  SleepFn sleep_;
  LogFn   log_;

  PhaseValues<Butterworth2nd<float>> applited_voltage_filters_;
  PhaseValues<Butterworth2nd<float>> feedback_voltage_filters_;

  float filter_cutoff_freq_hz_         = 2500.f;
  float filter_cutoff_freq_hz_current_ = 2500.f;
  float filter_cutoff_freq_hz_vel_     = 100.f;
  float filter_cutoff_freq_hz_fb_      = 500.f;

  float calibration_scan_speed_    = 0.25f * static_cast<float>(M_PI);
  float calibration_scan_distance_ = 0.5f  * static_cast<float>(M_PI);
  int   calibration_dir_           = 1;

  Angle encoder_angle{0, 0.f};
  int   pos_sensor_dir_   = 1;
  float shaft_angle_      = 0.f;
  float shaft_velocity_   = 0.f;
  float e_ang_offset_     = 0.f;
  float electrical_angle_ = 0.f;

  DiscreteFilter<float, float> pos_filter_{{1.f}, {}};
  DiscreteFilter<float, float> vel_filter_{{10000.f, -10000.f}, {}};
  Butterworth2nd<float> vel_filter_cutoff_ = Butterworth2nd<float>(100.f, 10000.f);

  float cogging_offset_ = 0.f;
  float target_         = 0.f;

  float open_loop_shaft_angle_    = 0.f;
  float open_loop_shaft_velocity_ = 0.f;

  PhaseValues<float>      phase_currents_{0.f, 0.f, 0.f};
  QuadDirectValues<float> quaddirect_currents_{0.f, 0.f};

  bool feedforward_enable_ = true;
  bool feedback_enable_    = true;
  bool back_emf_enable_    = true;

  float Kp_ = 0.f;
  float Ki_ = 0.f;

  QuadDirectValues<float> last_error_  {0.f, 0.f};
  QuadDirectValues<float> last_command_{0.f, 0.f};

  DiscreteFilter<QuadDirectValues<float>, float> feedback_;

  PhaseValues<float> last_desr_phase_currents_{0.f, 0.f, 0.f};
  PhaseValues<float> last_desr_phase_voltages_{0.f, 0.f, 0.f};
  PhaseValues<float> last_phase_volts_        {0.f, 0.f, 0.f};

  ControllerMode ctrl_mode_ = ControllerMode::DISABLE;

  int   control_period_us_ = 100;
  float control_period_s_  = 100.f * 1e-6f;
  float control_freq_hz_   = 10000.f;

  float MAX_VOLT_ = 3.f;

  bool anticog_enable_      = false;
  bool anticog_volt_enable_ = false;
  std::function<float(float)>              torque_mapper_ = [](float) -> float { return 0.f; };
  std::function<PhaseValues<float>(float)> volt_mapper_   = [](float) -> PhaseValues<float> { return {0.f, 0.f, 0.f}; };

  PhaseValues<float> feedforward(QuadDirectValues<float> desr_current)
  {
    PhaseValues<float> desr_phase_currents = quaddirect_to_phases<float>(desr_current, electrical_angle_);

    float A = 2.f * motor_.phase_L / control_period_s_ + motor_.phase_R;
    float B = -2.f * motor_.phase_L / control_period_s_ + motor_.phase_R;

    PhaseValues<float> desr_phase_voltages =
      A * desr_phase_currents + B * last_desr_phase_currents_ - last_desr_phase_voltages_;

    last_desr_phase_voltages_ = desr_phase_voltages;
    last_desr_phase_currents_ = desr_phase_currents;
    return desr_phase_voltages;
  }

  PhaseValues<float> feedback(QuadDirectValues<float> desr_current)
  {
    QuadDirectValues<float> error = desr_current - quaddirect_currents_;
    auto fb_phs_v = quaddirect_to_phases<float>(feedback_.update(error), electrical_angle_);
    return filter_feedback_voltages(fb_phs_v);
  }

  PhaseValues<float> back_emf_decoupler()
  {
    auto bemf_volt = std::clamp(
      0.5f * motor_.kV * shaft_velocity_, -motor_.kV * 300.f, motor_.kV * 300.f);
    return quaddirect_to_phases<float>({bemf_volt, 0.f}, electrical_angle_);
  }

  float get_eangle(float mech_ang) const
  {
    return normalize_angle(static_cast<float>(motor_.pole_pairs) * mech_ang - e_ang_offset_);
  }

  PhaseValues<float> center_phase_voltages(PhaseValues<float> phase_volts) const
  {
    float _min = std::min({phase_volts.a, phase_volts.b, phase_volts.c});
    auto  new_volts = phase_volts - PhaseValues<float>{_min, _min, _min};
    float _max = std::max({new_volts.a, new_volts.b, new_volts.c});
    float ratio = (_max > MAX_VOLT_) ? MAX_VOLT_ / _max : 1.f;
    return new_volts * ratio;
  }

  PhaseValues<float> filter_phase_voltages(PhaseValues<float> phase_volts)
  {
    return {applited_voltage_filters_.a.update(phase_volts.a),
            applited_voltage_filters_.b.update(phase_volts.b),
            applited_voltage_filters_.c.update(phase_volts.c)};
  }

  PhaseValues<float> filter_feedback_voltages(PhaseValues<float> phase_volts)
  {
    return {feedback_voltage_filters_.a.update(phase_volts.a),
            feedback_voltage_filters_.b.update(phase_volts.b),
            feedback_voltage_filters_.c.update(phase_volts.c)};
  }
};

#endif // NUCONTROL_CORE_BRUSHLESS_CONTROLLER_HPP
