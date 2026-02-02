#ifndef BRUSHLESS_CONTROLLER_HPP
#define BRUSHLESS_CONTROLLER_HPP
#include <TeensyTimerTool.h>
#include "driver.hpp"
#include "current_sense.hpp"
#include "transformations.hpp"
#include "encoder.hpp"
#include "discrete_filter.hpp"
#include "motors.hpp"
#include "anticogging_mapper.hpp"

enum ControllerMode
{
  DISABLE,
  OPEN_LOOP_VELOCITY,
  TORQUE,
};

class BrushlessController
{
public:
  BrushlessController() = default;
  ~BrushlessController() = default;

  BrushlessController(
    MotorParameters motor,
    BrushlessDriver & motor_driver,
    InlineCurrentSensorPackage & current_sensors,
    AbsoluteEncoder & pos_sensor)
  : motor_(motor),
    driver_(motor_driver),
    cs_(current_sensors),
    position_sensor_(pos_sensor),
    ctrl_timer_(TeensyTimerTool::TCK),
    print_timer_(TeensyTimerTool::TCK),
    anticog_map_(default_anticog_map),
    anticog_volt_map_(default_anticog_volt_map)
  {
    Kp_ = motor_.phase_L * _2_PI_ * 25.f; // Ohms = V / A
    Ki_ = motor_.phase_R * _2_PI_ * 25.f; // Ohms * s = Vs / A
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

    feedback_voltage_filters_.a =
      Butterworth2nd<float>(filter_cutoff_freq_hz_fb_, control_freq_hz_);
    feedback_voltage_filters_.b =
      Butterworth2nd<float>(filter_cutoff_freq_hz_fb_, control_freq_hz_);
    feedback_voltage_filters_.c =
      Butterworth2nd<float>(filter_cutoff_freq_hz_fb_, control_freq_hz_);
  }

  void set_control_mode(ControllerMode ctrl_mode)
  {
    ctrl_mode_ = ctrl_mode;
  }

  void start_control(int control_period_us, bool use_internal_timer = true)
  {
    control_period_us_ = control_period_us;
    control_period_s_ = control_period_us_ * 1e-6;
    control_freq_hz_ = 1.f / control_period_s_;

    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_current_, filter_cutoff_freq_hz_fb_);

    update_sensors();

    pos_filter_.reset(shaft_angle_);
    vel_filter_.reset(shaft_angle_);
    vel_filter_cutoff_.reset();

    shaft_velocity_ = 0.f;

    last_error_ = QuadDirectValues<float>{0.f, 0.f};
    last_command_ = QuadDirectValues<float>{0.f, 0.f};

    last_desr_phase_currents_ = PhaseValues<float>{0.f, 0.f, 0.f};
    last_desr_phase_voltages_ = PhaseValues<float>{0.f, 0.f, 0.f};
    driver_.enable();

    if (use_internal_timer) {
      internal_timer_on_ = true;
      ctrl_timer_.begin(
        [this] {
          control_step();
        }, control_period_us_);
    }
  }

  void stop_control()
  {
    if(internal_timer_on_){
      internal_timer_on_ = false;
      ctrl_timer_.stop();
      }
    driver_.disable();
  }

  void start_print(int print_period_ms)
  {
    print_timer_.begin(
      [this] {
        printer();
      }, print_period_ms * 1000);
  }

  void set_feedback_control(DiscreteFilter<QuadDirectValues<float>, float> fb_filter) { feedback_ = fb_filter; }

  void set_velocity_filter(DiscreteFilter<float, float> vel_filter) { vel_filter_ = vel_filter; }

  void set_position_filter(DiscreteFilter<float, float> pos_filter) { pos_filter_ = pos_filter; }

  void stop_print()
  {
    print_timer_.stop();
  }

  bool init_components()
  {
    auto ret_d = driver_.init();
    auto ret_cs = cs_.init_sensors();
    Serial.println(ret_d);
    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_current_, filter_cutoff_freq_hz_fb_);

    return ret_d & ret_cs;

  }

  bool align_sensors(bool e_angle_align = true)
  {

    auto ret = cs_.align_sensors(driver_);

    if(!e_angle_align) {
      return ret;
    }

    delay(1000);

    // Reset open loop angle and velocity 
    encoder_offset_ = 0.f;
    open_loop_shaft_angle_ = 0.f;
    open_loop_shaft_velocity_ = 0.f;

    set_control_mode(ControllerMode::OPEN_LOOP_VELOCITY);
    target_ = calibration_scan_speed_;

    start_control(1000, false);

    update_sensors();

    float init_ang = encoder_angle.get_full_angle();
    float enc_sum = 0.f;
    int steps = 0;

    while(open_loop_shaft_angle_ < calibration_scan_distance_){
      update_sensors();
      enc_sum += encoder_angle.get_angle();
      steps++;
      update_control();
      delay(1);
    }

    target_ = 0.f;
    update_control();
    stop_control();

    Serial.println("Forward Move Complete");
    Serial.flush();

    if(encoder_angle.get_full_angle() > init_ang + 0.5f * calibration_scan_distance_){
      pos_sensor_dir_ = 1;
      Serial.println("Direction is: 1");
    } else {
      if(encoder_angle.get_full_angle() < init_ang - 0.5f * calibration_scan_distance_){
        pos_sensor_dir_ = -1;
        Serial.println("Direction is: -1");
      }
    else {
      Serial.println("Error: Sensor did not report motion");
      Serial.flush();
      return false;
    }
  }

    start_control(1000, false);
    open_loop_shaft_angle_ = 0.f;
    open_loop_shaft_velocity_ = 0.f;
    target_ = -calibration_scan_speed_;

    while(open_loop_shaft_angle_ > -calibration_scan_distance_){
      update_sensors();
      enc_sum += encoder_angle.get_angle();
      steps++;
      update_control();
      delay(1);
    }

    Serial.println("Backwards Move Complete");
    Serial.flush();

    target_ = 0.f;
    update_control();
    stop_control();

    set_control_mode(ControllerMode::DISABLE);

    encoder_offset_ = pos_sensor_dir_ * enc_sum / steps;

    Serial.println(encoder_offset_);

    return ret;
  }

  float get_encoder_offset() const { return encoder_offset_; }
  float get_shaft_angle() const { return shaft_angle_; }
  float get_shaft_radians() const { return normalize_angle(shaft_angle_); }
  float get_encoder_angle() const { return encoder_angle.get_full_angle(); }
  float get_encoder_radians() const { return encoder_angle.get_angle(); }
  float get_shaft_velocity() const { return shaft_velocity_; }

  void set_encoder_direction(int dir) {
    if(dir == 1){ pos_sensor_dir_= 1; return;}
    if(dir == -1){ pos_sensor_dir_= -1; return;}
    else{
      Serial.println("Error: Invalid Direction. Please use 1 or -1");
      return;
    }
    
  }
  void set_encoder_offset(float offset) { encoder_offset_ = offset; }
  void set_calibration_scan_speed(float w) { calibration_scan_speed_ = w;}
  void set_calibration_scan_range (float rads) {calibration_scan_distance_ = rads;}

  void enable_anticog(const std::vector<float> & map)
  {
    disable_anticog();
    anticog_enable_ = true;
    anticog_map_ = map;
  }

  void enable_anticog(const PhaseValues<std::vector<float>> & map)
  {
    disable_anticog();
    anticog_volt_enable_ = true;
    anticog_volt_map_ = map;
  }

  void disable_anticog()
  {
    anticog_volt_enable_ = false;
    anticog_enable_ = false;
    anticog_map_ = default_anticog_map;
    anticog_volt_map_ = default_anticog_volt_map;
  }

  void set_feedforward_state(bool state){ feedforward_enable_ = state; }

  void set_feedback_state(bool state){ feedback_enable_ = state; }

  void set_back_emf_comp_state(bool state){ back_emf_enable_ = state; }

  void set_cogging_offset(float offset){ cogging_offset_ = offset;}

  void set_target(float target){ target_ = target; }


  PhaseValues<float> get_last_phasevolts() const { return last_phase_volts_; }

  void update_sensors()
  {
    encoder_angle.update_angle(position_sensor_.read());
    shaft_angle_ = pos_filter_.update(pos_sensor_dir_ * encoder_angle.get_full_angle());
    electrical_angle_ = get_eangle(shaft_angle_);
    shaft_velocity_ = vel_filter_cutoff_.update(vel_filter_.update(shaft_angle_));
    phase_currents_ = cs_.get_phase_currents(true);
    quaddirect_currents_ = phases_to_quaddirect<float>(phase_currents_, electrical_angle_);
  }

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
          if (anticog_enable_) { target_ += get_cogging_torque(shaft_angle_ - cogging_offset_, anticog_map_); }

          // Convert requested torque into a current request
          float requested_current = target_ / motor_.kT;

          // Let's limit to the stall current of the motor
          // We don't know user's intentions so we can't just limit to safe current
          requested_current = std::clamp(requested_current, -motor_.MAX_CURRENT, motor_.MAX_CURRENT);

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
            filtered_ctrl_volts += get_cogging_voltage(shaft_angle_ - cogging_offset_, anticog_volt_map_);
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

private:
  MotorParameters motor_;
  BrushlessDriver & driver_;
  InlineCurrentSensorPackage & cs_;
  AbsoluteEncoder & position_sensor_;

  PhaseValues<Butterworth2nd<float>> applited_voltage_filters_;
  PhaseValues<Butterworth2nd<float>> feedback_voltage_filters_;

  /// \note: General cutoff frequency
  float filter_cutoff_freq_hz_ = 2500.f;

  /// \note: Current sensor specific cutoff
  float filter_cutoff_freq_hz_current_ = 2500.f;

  /// \note: Velocity estimator specific cutoff
  float filter_cutoff_freq_hz_vel_ = 100.f;

  float filter_cutoff_freq_hz_fb_ = 500.f;

  float calibration_scan_speed_ = PI;
  float calibration_scan_distance_ = 0.25f * PI;


  /// \note: Position / State Variables

  Angle encoder_angle{0, 0.f};
  int pos_sensor_dir_ = 1;
  float shaft_angle_ = 0.f; // radians
  float shaft_velocity_ = 0.f; // rad /s
  float encoder_offset_ = 0.f; // radians
  float electrical_angle_ = 0.f; //radians

  DiscreteFilter<float, float> pos_filter_{{1.f}, {}};
  DiscreteFilter<float, float> vel_filter_{{10000.f, -10000.f}, {}};
  Butterworth2nd<float> vel_filter_cutoff_ = Butterworth2nd<float>(100.f, 10000.f);

  float cogging_offset_ = 0.f; //radians

  float target_; // Units depend on control mode, either rad /s or Nm

  /// \note: Open loop variables
  float open_loop_shaft_angle_ = 0.f;
  float open_loop_shaft_velocity_ = 0.f;


  PhaseValues<float> phase_currents_{0.f, 0.f, 0.f};
  QuadDirectValues<float> quaddirect_currents_{0.f, 0.f};

  bool feedforward_enable_ = true;
  bool feedback_enable_ = true;
  bool back_emf_enable_ = true;

  bool internal_timer_on_;

  /// \note: Feedback controller variables
  float Kp_;
  float Ki_;

  QuadDirectValues<float> last_error_{0.f, 0.f};
  QuadDirectValues<float> last_command_{0.f, 0.f};

  DiscreteFilter<QuadDirectValues<float>, float> feedback_;

  /// \note: Feedforward controller variables
  PhaseValues<float> last_desr_phase_currents_{0.f, 0.f, 0.f};
  PhaseValues<float> last_desr_phase_voltages_{0.f, 0.f, 0.f};

  PhaseValues<float> last_phase_volts_{0.f, 0.f, 0.f};

  ControllerMode ctrl_mode_ = ControllerMode::DISABLE;

  TeensyTimerTool::PeriodicTimer ctrl_timer_;
  TeensyTimerTool::PeriodicTimer print_timer_;

  int control_period_us_ = 100; //us
  float control_period_s_ = 100.f * 1e-6f; // s
  float control_freq_hz_ = 10000.f;

  float MAX_VOLT_ = 3.f;

  bool anticog_enable_ = false;
  std::vector<float> & anticog_map_;
  bool anticog_volt_enable_ = false;
  PhaseValues<std::vector<float>> & anticog_volt_map_;

  bool debug_print_ = true;

  void debug_print(auto msg)
  {
    if (!debug_print_) {return;}
    Serial.print(msg);
  }
  void debug_println(auto msg)
  {
    if (!debug_print_) {return;}
    Serial.println(msg);
  }

  void control_step()
  {
    update_sensors();
    update_control();
  }

  PhaseValues<float> feedforward(QuadDirectValues<float> desr_current)
  {
    PhaseValues<float> desr_phase_currents = quaddirect_to_phases<float>(desr_current, electrical_angle_);

    float A = 2.f * motor_.phase_L / control_period_s_ + motor_.phase_R;
    float B = -2.f * motor_.phase_L / control_period_s_ + motor_.phase_R;

    PhaseValues<float> desr_phase_voltages = A * desr_phase_currents + B * last_desr_phase_currents_ - last_desr_phase_voltages_;

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
      0.5f * motor_.kV * shaft_velocity_, -motor_.kV * 300.f,
      motor_.kV * 300.f);

    auto bemf =
      quaddirect_to_phases<float>({bemf_volt, 0.f}, electrical_angle_);
    return bemf;
  }

  void printer()
  {
    Serial.println(shaft_velocity_);
  }

  float get_eangle(float mech_ang) const
  {
    return normalize_angle(static_cast<float>(motor_.pole_pairs) * (mech_ang - encoder_offset_));
  }

  PhaseValues<float> center_phase_voltages(PhaseValues<float> phase_volts) const
  {
    float _min = min(phase_volts.a, min(phase_volts.b, phase_volts.c));

    PhaseValues<float> offset_volts{_min, _min, _min};

    auto new_volts = phase_volts - offset_volts;

    float _max = max(phase_volts.a, max(phase_volts.b, phase_volts.c));
    float ratio = 1.f;
    if (_max > MAX_VOLT_) {
      ratio = MAX_VOLT_ / _max;
    }
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

#endif
