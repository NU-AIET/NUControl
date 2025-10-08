#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP
#include <TeensyTimerTool.h>
#include "driver.hpp"
#include "current_sense.hpp"
#include "transformations.hpp"
#include "spi_encoder.hpp"
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
    SPIEncoder & pos_sensor)
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
    set_feedback_filter(PIController<QuadDirectValues<float>>(Kp_, Ki_, control_period_s_));
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

    last_vel_angle_ = 0.f;
    shaft_velocity_ = 0.f;
    loops_since_tick_ = -1;

    last_error_ = QuadDirectValues<float>{0.f, 0.f};
    last_command_ = QuadDirectValues<float>{0.f, 0.f};

    last_desr_phase_currents_ = PhaseValues<float>{0.f, 0.f, 0.f};
    last_desr_phase_voltages_ = PhaseValues<float>{0.f, 0.f, 0.f};

    start_time_ = micros() * 1e-6f;
    driver_.enable();

    if (use_internal_timer) {
      ctrl_timer_.begin(
        [this] {
          control_step();
        }, control_period_us_);
    }
  }

  void stop_control()
  {
    ctrl_timer_.stop();
    driver_.disable();
  }

  void start_print(int print_period_ms)
  {
    print_timer_.begin(
      [this] {
        printer();
      }, print_period_ms * 1000);
  }

  void set_feedback_filter(DiscreteFilter<QuadDirectValues<float>, float> fb_filter)
  {
    feedback_ = fb_filter;
  }

  void stop_print()
  {
    print_timer_.stop();
  }

  bool init_components()
  {
    auto ret_d = driver_.init();
    auto ret_cs = cs_.init_sensors();
    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_current_, filter_cutoff_freq_hz_fb_);

    return ret_d & ret_cs;

  }

  bool align_sensors(int dir = 0, bool skip_eang = false)
  {
    e_ang_offset_ = 0.f;

    auto ret = cs_.align_sensors(driver_);

    delay(1000);
    open_loop_shaft_angle_ = 1.5f * PI;
    open_loop_shaft_velocity_ = 0.f;

    if (dir == 0) {

      // float inital_shaft_angle = shaft_angle_.get_full_angle();
      set_control_mode(ControllerMode::OPEN_LOOP_VELOCITY);
      target_ = 0.25 * PI;
      start_control(control_period_us_, false);
      ctrl_timer_.begin(
        [this]
        {
          control_step();
          if (open_loop_shaft_angle_ > 3.5 * PI) {stop_control();}
        }, control_period_us_);
      delay(100);

      auto pos_spin_ang = shaft_angle_.get_full_angle();

      target_ = -0.25 * PI;
      start_control(control_period_us_, false);
      ctrl_timer_.begin(
        [this]
        {
          control_step();
          if (open_loop_shaft_angle_ < 1.5 * PI) {stop_control();}
        }, control_period_us_);
      delay(100);

      auto neg_spin_ang = shaft_angle_.get_full_angle();

      const auto displacement = fabs(pos_spin_ang - neg_spin_ang);

      if (displacement < 0.05) {
        Serial.println("Sensor did not report motion");
        return false;
      }

      if (pos_spin_ang < neg_spin_ang) {
        pos_sensor_dir_ *= -1;
      }
      Serial.print("Sensor Direction Is: ");
      Serial.println(pos_sensor_dir_);

      auto pp_check = !((fabs(displacement * motor_.pole_pairs - _2_PI_)) > 0.5);
      if (pp_check) {
        Serial.print("Polepairs estimated at different count: ");
        Serial.println(static_cast<int>(_2_PI_ / displacement));
      }


      shaft_velocity_ = 0;
    } else {
      pos_sensor_dir_ = dir;
    }

    shaft_angle_.direction = pos_sensor_dir_;
    // One last call to flush anything in case we flipped directions
    update_sensors();

    if (!skip_eang) {
      // Find zero electrical angle;

      // Send voltage to pull the driver towards the zero electrical angle
      // May perform worse on motors with lots of friction, cogging, or other impedances
      driver_.enable();
      auto phase_volts = quaddirect_to_phases<float>(
        {motor_.phase_R * motor_.SAFE_CURRENT, 0.f},
        1.5f * PI);
      driver_.set_phase_voltages(center_phase_voltages(phase_volts));
      delay(700);
      // do{
      //   update_sensors();
      //   delay(10);
      //   Serial.println(shaft_velocity_);
      // }while(abs(shaft_velocity_) > 0.01f);
      for (size_t i = 0; i < 500; ++i) {
        update_sensors();
      }
      delay(10);
      e_ang_offset_ = 0.f;
      e_ang_offset_ = get_eangle(shaft_angle_.get_angle());
      driver_.set_phase_voltages({0.f, 0.f, 0.f});
      delay(300);
      driver_.disable();

      Serial.print("Zero Electrical Angle: ");
      Serial.println(e_ang_offset_);
    }

    return ret;
  }

  void set_e_angle_offset(float e_ang)
  {
    e_ang_offset_ = e_ang;
  }

  float get_e_angle_offset() const
  {
    return e_ang_offset_;
  }

  float get_shaft_angle() const
  {
    return shaft_angle_.get_full_angle();
  }

  float get_shaft_radians() const
  {
    return shaft_angle_.get_angle();
  }

  float get_shaft_velocity() const
  {
    return shaft_velocity_;
  }

  void enable_anticog(const std::vector<float> & map)
  {
    disable_anticog();
    anticog_ = true;
    anticog_map_ = map;

  }

  void enable_anticog(const PhaseValues<std::vector<float>> & map)
  {
    disable_anticog();
    anticog_volt_ = true;
    anticog_volt_map_ = map;
  }

  void disable_anticog()
  {
    anticog_volt_ = false;
    anticog_ = false;
    anticog_map_ = default_anticog_map;
    anticog_volt_map_ = default_anticog_volt_map;
  }

  void set_cogging_offset()
  {
    cogging_offset_ = position_sensor_.read();

  }

  PhaseValues<float> get_last_phasevolts() const
  {
    return last_phase_volts_;
  }

  void update_sensors()
  {
    // auto start = micros();
    // while(start - micros() < 25)
    // {
    //   yield();
    // }

    // This is only tripped on the first loop since control has started.
    // We need to do some cleanup to make sure our filters won't delay new inputs
    if (loops_since_tick_ == -1) {
      mechanical_angle_ = position_sensor_.read();
      // shaft_angle_.filter_.reset(pos, pos);
      shaft_angle_.update_angle(mechanical_angle_);
      last_vel_angle_ = shaft_angle_.get_full_angle();
      // vel_filter_.reset();
      loops_since_tick_++;

    } else {

      // Position first
      mechanical_angle_ = position_sensor_.read();
      shaft_angle_.update_angle(mechanical_angle_);
      auto new_angle = shaft_angle_.get_full_angle();

      if (loops_since_tick_ == 2000) {
        float vel_hat = (new_angle - last_vel_angle_) /
          (static_cast<float>(2000) * control_period_s_ );
        last_vel_angle_ = new_angle;
        // shaft_velocity_ = vel_filter_.update(vel_hat);
        shaft_velocity_ = vel_hat;
        loops_since_tick_ = 0;
      }
    }

    loops_since_tick_++;

    // shaft_angles_.at(i_) = shaft_angle_.get_full_angle();
    // vel_angles_.at(i_) = last_vel_angle_;


    // vels_.at(i_) = shaft_velocity_;

    phase_currents_ = cs_.get_phase_currents(true);
    quaddirect_currents_ =
      phases_to_quaddirect<float>(phase_currents_, get_eangle(shaft_angle_.get_full_angle()));
  }

  void set_target(float target)
  {
    target_ = target;
  }

  void update_control()
  {
    switch (ctrl_mode_) {
      case ControllerMode::DISABLE:
        return;
      case ControllerMode::OPEN_LOOP_VELOCITY:
        open_loop_shaft_angle_ += target_ * control_period_s_;
        open_loop_shaft_velocity_ = target_;
        {
          // In this case, the target is a desired shaft velocity
          // Everything is an estimate in open loop

          float back_emf = motor_.kV * target_;

          auto phase_volts =
            quaddirect_to_phases<float>(
            {motor_.phase_R * motor_.SAFE_CURRENT + back_emf, 0.f}, get_eangle(
              open_loop_shaft_angle_));
          auto cntr_volts = center_phase_voltages(phase_volts);
          driver_.set_phase_voltages(cntr_volts);
        }
        return;

      case ControllerMode::TORQUE:
        {

          // Convert requested torque into a current request
          float requested_current = target_ / motor_.kT;

          if (anticog_) {
            requested_current +=
              get_cogging_torque(mechanical_angle_ - cogging_offset_, anticog_map_) / motor_.kT;
          }

          // Let's limit to the stall current of the motor
          // We don't know user's intentions so we can't just limit to safe current
          requested_current =
            std::clamp(requested_current, -motor_.MAX_CURRENT, motor_.MAX_CURRENT);

          // Generate desired current in QD frame, D = 0;
          QuadDirectValues<float> desr_current{requested_current, 0};


          // Pump controllers
          auto ff_volts = feedforward(desr_current);
          // auto fb_volts = feedback(desr_current);
          auto bemf_volts = back_emf_decoupler();

          auto ctrl_volts = filter_phase_voltages(ff_volts + bemf_volts);

          if(anticog_volt_){
            ctrl_volts += get_cogging_voltage(mechanical_angle_ - cogging_offset_, anticog_volt_map_);
          }

          const auto dr_volts = center_phase_voltages(ctrl_volts) + PhaseValues<float>{1.f, 1.f, 1.f};
          
          last_phase_volts_ = dr_volts;

          // desr_voltage_.at(i_) = dr_volts;

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
  SPIEncoder & position_sensor_;


  /// \note: Filters
  // Unused
  // Butterworth2nd<float> pos_filter_;
  // Butterworth2nd<float> vel_filter_;

  PhaseValues<Butterworth2nd<float>> applited_voltage_filters_;
  PhaseValues<Butterworth2nd<float>> feedback_voltage_filters_;

  /// \note: General cutoff frequency
  float filter_cutoff_freq_hz_ = 2500.f;

  /// \note: Current sensor specific cutoff
  float filter_cutoff_freq_hz_current_ = 2500.f;

  /// \note: Velocity estimator specific cutoff
  float filter_cutoff_freq_hz_vel_ = 100.f;

  float filter_cutoff_freq_hz_fb_ = 500.f;


  /// \note: Position / State Variables

  float mechanical_angle_ = 0.f;

  int pos_sensor_dir_ = 1;
  Angle shaft_angle_{0, 0.f};
  int loops_since_tick_ = -1;
  float shaft_velocity_ = 0.f; // rad /s
  float last_vel_angle_ = 0.f;

  float e_ang_offset_ = 0.f; //rad = How much to subtract from mech ang to align w/ electical angle

  float cogging_offset_ = 0.f;

  float target_; // Units depend on control mode, either rad /s or Nm

  /// \note: Open loop variables
  float open_loop_shaft_angle_ = 0.f;
  float open_loop_shaft_velocity_ = 0.f;


  PhaseValues<float> phase_currents_{0.f, 0.f, 0.f};
  QuadDirectValues<float> quaddirect_currents_{0.f, 0.f};

  /// \note: Feedback controller variables
  float Kp_;
  float Ki_;

  QuadDirectValues<float> last_error_{0.f, 0.f};
  QuadDirectValues<float> last_command_{0.f, 0.f};

  DiscreteFilter<QuadDirectValues<float>, float> feedback_;

  /// \note: Feedforward controller variables
  PhaseValues<float> last_desr_phase_currents_{0.f, 0.f, 0.f};
  PhaseValues<float> last_desr_phase_voltages_{0.f, 0.f, 0.f};
  PhaseValues<float> desr_phase_current_dot_{0.f, 0.f, 0.f};

  PhaseValues<float> last_phase_volts_{0.f, 0.f, 0.f};


  ControllerMode ctrl_mode_ = ControllerMode::DISABLE;

  TeensyTimerTool::PeriodicTimer ctrl_timer_;
  TeensyTimerTool::PeriodicTimer print_timer_;

  int control_period_us_ = 100; //us
  float control_period_s_ = 100.f * 1e-6f; // s
  float control_freq_hz_ = 10000.f;

  float MAX_VOLT_ = 3.f;

  bool anticog_ = false;
  std::vector<float> & anticog_map_;
  bool anticog_volt_ = false;
  PhaseValues<std::vector<float>> & anticog_volt_map_;

  bool debug_print_ = true;

  const size_t max_size = 1;

  std::vector<float> time_ = std::vector<float>(max_size, 0.f);
  std::vector<float> ref_current_ = std::vector<float>(max_size, 0.f);
  std::vector<QuadDirectValues<float>> meas_current_ = std::vector<QuadDirectValues<float>>(
    max_size, {0.f, 0.f});
  std::vector<PhaseValues<float>> desr_voltage_ = std::vector<PhaseValues<float>>(
    max_size, {0.f,
      0.f, 0.f});
  std::vector<float> vels_ = std::vector<float>(max_size, 0.f);
  std::vector<float> shaft_angles_ = std::vector<float>(max_size, 0.f);
  std::vector<float> vel_angles_ = std::vector<float>(max_size, 0.f);

  size_t i_ = 0;
  float start_time_;
  int max_loops_ = 15;
  int loops_ = 0;

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

  float afreq(float t)
  {
    return 1000.f * _2_PI_;
  }

  void control_step()
  {
    // float now = micros() * 1e-6f - start_time_;
    // float w = afreq(now);
    // float req_curr = 0.f; //0.5f * sinf(w * now);
    // target_ = motor_.kT * req_curr;
    // time_.at(i_) = now;
    // ref_current_.at(i_) = req_curr;
    update_sensors();
    update_control();

    if(loops_since_tick_ == -1) {
      loops_since_tick_++; // Bump back up to 0
    }

    if(loops_since_tick_ == 1000) {
      loops_since_tick_ = 0;
    }
    loops_since_tick_++;
    // Serial.println(shaft_velocity_);
    // meas_current_.at(i_) = quaddirect_currents_;

    // ++i_;

    // if (i_ >= max_size) {
    //   stop_control();
    //   data_out();
    // }
  }

  void data_out()
  {
    Serial.println("=====");
    Serial.flush();
    delay(10);
    for (size_t j = 0; j < i_; ++j) {
      Serial.print(time_.at(j), 6);
      Serial.print("\t");
      Serial.print(ref_current_.at(j), 6);
      Serial.print("\t");
      Serial.print(meas_current_.at(j).q, 6);
      Serial.print("\t");
      Serial.print(meas_current_.at(j).d, 6);
      Serial.print("\t");
      Serial.print(vels_.at(j), 6);
      Serial.print("\t");
      Serial.print(shaft_angles_.at(j), 6);
      Serial.print("\t");
      Serial.print(vel_angles_.at(j), 6);
      Serial.print("\t");
      Serial.print(desr_voltage_.at(j).a, 6);
      Serial.print("\t");
      Serial.print(desr_voltage_.at(j).b, 6);
      Serial.print("\t");
      Serial.println(desr_voltage_.at(j).c, 6);
      Serial.flush();
    }
    i_ = 0;
    delay(1);
    Serial.println("=====");
    Serial.flush();
    delay(10);
    loops_++;
    Serial.print("Loop #");
    Serial.print(loops_);
    Serial.println(" finished!");
    if (loops_ < max_loops_) {
      i_ = 0;
      start_time_ = micros() * 1e-6f;
      start_control(100);
    } else {
      exit(1);
    }

  }

  PhaseValues<float> feedback(QuadDirectValues<float> desr_current)
  {

    auto e_ang = get_eangle(shaft_angle_.get_full_angle());

    QuadDirectValues<float> error = desr_current - quaddirect_currents_;

    auto fb_phs_v = quaddirect_to_phases<float>(feedback_.update(error), e_ang);

    return filter_feedback_voltages(fb_phs_v);
  }

  PhaseValues<float> feedforward(QuadDirectValues<float> desr_current)
  {
    PhaseValues<float> desr_phase_currents =
      quaddirect_to_phases<float>(desr_current, get_eangle(shaft_angle_.get_full_angle()));

    // if(loops_since_tick_ == -1) {
    //   last_desr_phase_currents_ = PhaseValues<float>{0.f, 0.f, 0.f};
    //   desr_phase_current_dot_ = PhaseValues<float>{0.f, 0.f, 0.f};
    // }
    // if(loops_since_tick_ % 100 == 0){
    //   desr_phase_current_dot_ = (desr_phase_currents - last_desr_phase_currents_) * (1 / (100.f * control_period_s_));
    // }

    float A = 2.f * motor_.phase_L / control_period_s_ + motor_.phase_R;
    float B = -2.f * motor_.phase_L / control_period_s_ + motor_.phase_R;

    PhaseValues<float> desr_phase_voltages = A * desr_phase_currents + B *
      last_desr_phase_currents_ - last_desr_phase_voltages_;

    // PhaseValues<float> desr_phase_voltages = motor_.phase_R * desr_phase_currents + motor_.phase_L * desr_phase_current_dot_;

    last_desr_phase_voltages_ = desr_phase_voltages;
    last_desr_phase_currents_ = desr_phase_currents;
    return desr_phase_voltages;
  }

  PhaseValues<float> back_emf_decoupler()
  {
    auto bemf_volt = std::clamp(
      0.5f * motor_.kV * shaft_velocity_, -motor_.kV * 300.f,
      motor_.kV * 300.f);

    auto bemf =
      quaddirect_to_phases<float>({bemf_volt, 0.f}, get_eangle(shaft_angle_.get_full_angle()));
    return bemf;
  }

  void printer()
  {
    // Serial.print(1000.f * target_);
    // Serial.print("\t");
    // Serial.print(1000.f * quaddirect_currents_.q * motor_.kT);
    // Serial.print("\t");
    // Serial.println(shaft_angle_.get_full_angle());
    Serial.println(shaft_velocity_);
  }


  float get_eangle(float mech_ang) const
  {
    return normalize_angle(static_cast<float>(motor_.pole_pairs) * mech_ang - e_ang_offset_);
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
    return {applited_voltage_filters_.a.update(phase_volts.a), applited_voltage_filters_.b.update(
        phase_volts.b), applited_voltage_filters_.c.update(phase_volts.c)};
  }
  PhaseValues<float> filter_feedback_voltages(PhaseValues<float> phase_volts)
  {
    return {feedback_voltage_filters_.a.update(phase_volts.a),
      feedback_voltage_filters_.b.update(phase_volts.b),
      feedback_voltage_filters_.c.update(phase_volts.c)};
  }


};


class BrushedController
{
public:
  BrushedController() = default;
  ~BrushedController() = default;

  BrushedController(MotorParameters motor, BrushedDriver & motor_driver)
  : motor_(motor),
    driver_(motor_driver),
    ctrl_timer_(TeensyTimerTool::TMR4),
    print_timer_(TeensyTimerTool::TCK)
  {}


  void set_control_mode(ControllerMode mode)
  {
    control_mode_ = mode;
  }

  void set_target(float target)
  {
    target_ = target;
  }

  void update_control()
  {
    switch (control_mode_) {
      case ControllerMode::OPEN_LOOP_VELOCITY:
        {
          auto v_d = motor_.kV * target_;
          driver_.set_voltage(center_voltage({v_d, 0}));
          /* code */
          return;
        }

      default:
        return;
    }

  }

private:
  MotorParameters motor_;
  BrushedDriver & driver_;

  TeensyTimerTool::PeriodicTimer ctrl_timer_;
  TeensyTimerTool::PeriodicTimer print_timer_;

  ControllerMode control_mode_ = ControllerMode::DISABLE;

  float target_ = 0.f;
  float MAX_VOLTAGE_ = 24.f;

  std::pair<float, float> center_voltage(std::pair<float, float> volts)
  {
    auto min_ = min(volts.first, volts.second);

    float ratio = 1.f;
    auto max_ = max(volts.first, volts.second);
    if (max_ > MAX_VOLTAGE_) {
      ratio = MAX_VOLTAGE_ / max_;
    }
    return {ratio * (volts.first - min_) + 1, ratio * (volts.second - min_) + 1};

  }


};

#endif
