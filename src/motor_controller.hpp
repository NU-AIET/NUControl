#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP
#include <TeensyTimerTool.h>

#include "driver.hpp"
#include "current_sense.hpp"
#include "transformations.hpp"
#include "spi_encoder.hpp"
#include "discrete_filter.hpp"

struct MotorParameters
{
  int pole_pairs;
  float phase_R;   // Ohm = Single Phase Resistance
  float phase_L;   // Ohm *s = Henry = Single Phase Inductance
  float SAFE_CURRENT;   // A = I can use this all the time and it will be okay
  float MAX_CURRENT;   // A = I can only use this for a short period
  float kT;   // Nm / A
  float kV;   // V / rad /s
};

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
    ctrl_timer_(TeensyTimerTool::TMR4),
    print_timer_(TeensyTimerTool::TCK)
  {
    Kp_ = motor_.phase_L * _2_PI_ * 200.f; // Ohms = V / A
    Ki_ = motor_.phase_R * _2_PI_ * 200.f; // Ohms * s = Vs / A
    MAX_VOLT_ = 1.5f * motor_.phase_R * motor_.MAX_CURRENT;
    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_vel_);
  }

  void set_filters(float cutoff_freq_hz, float cuttoff_freq_hz_vel)
  {
    filter_cutoff_freq_hz_ = cutoff_freq_hz;
    filter_cutoff_freq_hz_vel_ = cuttoff_freq_hz_vel;

    pos_filter_ = Butterworth2(300, control_freq_hz_);
    cs_.set_filters({filter_cutoff_freq_hz_current_, control_freq_hz_});
    vel_filter_ = Butterworth2(1000, control_freq_hz_);
    bemf_filter_= Butterworth2(1000, control_freq_hz_);
    applited_voltage_filters_.a = Butterworth2(cutoff_freq_hz, control_freq_hz_);
    applited_voltage_filters_.b = Butterworth2(cutoff_freq_hz, control_freq_hz_);
    applited_voltage_filters_.c = Butterworth2(cutoff_freq_hz, control_freq_hz_);
  }

  void set_control_mode(ControllerMode ctrl_mode)
  {
    ctrl_mode_ = ctrl_mode;
  }

  void start_control(int control_period_us)
  {
    control_period_us_ = control_period_us;
    control_period_s_ = control_period_us_ * 1e-6;
    control_freq_hz_ = 1.f / control_period_s_;
    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_vel_);

    last_error_ = QuadDirectValues<float>{0.f, 0.f};
    last_command_ = QuadDirectValues<float>{0.f, 0.f};

    last_desr_phase_currents_ = PhaseValues<float>{0.f, 0.f, 0.f};
    last_desr_phase_voltages_ = PhaseValues<float>{0.f, 0.f, 0.f};

    start_time_ = micros() * 1e-6f;
    driver_.enable();

    ctrl_timer_.begin(
      [this] {
        control_step();
      }, control_period_us_);
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

  void stop_print()
  {
    print_timer_.stop();
  }

  bool init_components()
  {
    auto ret_d = driver_.init();
    auto ret_cs = cs_.init_sensors();
    set_filters(filter_cutoff_freq_hz_, filter_cutoff_freq_hz_vel_);

    return ret_d & ret_cs;

  }

  bool align_sensors(bool skip_dir = false, bool skip_eang = false)
  {
    e_ang_offset_ = 0.f;

    auto ret = cs_.align_sensors(driver_);

    delay(1000);
    open_loop_shaft_angle_ = 1.5f * PI;
    open_loop_shaft_velocity_ = 0.f;

    if (!skip_dir) {

      float inital_shaft_angle = shaft_angle_.get_full_angle();
      set_control_mode(ControllerMode::OPEN_LOOP_VELOCITY);
      target_ = _2_PI_;
      start_control(control_period_us_);
      delay(100);
      while (open_loop_shaft_angle_ < 3.5 * PI) {}
      stop_control();
      delay(100);

      auto pos_spin_ang = shaft_angle_.get_full_angle();

      target_ = -_2_PI_;
      start_control(control_period_us_);
      delay(100);
      while (open_loop_shaft_angle_ > 1.5 * PI) {}
      stop_control();
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

      // One last call to flush anything in case we flipped directions
      update_sensors();
      shaft_velocity_ = 0;
    }

    if (!skip_eang) {
      // Find zero electrical angle;

      // Send voltage to pull the driver towards the zero electrical angle
      // May perform worse on motors with lots of friction, cogging, or other impedances
      driver_.enable();
      auto phase_volts = quaddirect_to_phases<float>({0.5f, 0.f}, 1.5f * PI);
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

  float get_shaft_angle() const
  {
    return last_shaft_angle_;
  }

  float get_shaft_velocity() const
  {
    return shaft_velocity_;
  }

  void update_sensors()
  {
    // Position first


    shaft_angle_.update_angle(pos_sensor_dir_ * position_sensor_.read());

    auto new_angle = pos_filter_.filter(shaft_angle_.get_full_angle());

    if(loops_since_tick_ == 100) {

      float vel_hat = (new_angle - last_vel_angle_) / (static_cast<float>(100) * control_period_s_ );
      last_vel_angle_ = new_angle;
      shaft_velocity_ = vel_filter_.filter(vel_hat);
      loops_since_tick_ = 0;
    }

    last_shaft_angle_ = new_angle;

    loops_since_tick_++;

    phase_currents_ = cs_.get_phase_currents(true);
    quaddirect_currents_ =
      phases_to_quaddirect<float>(phase_currents_, get_eangle(last_shaft_angle_));
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
          // May contradict other sensors
          auto phase_volts =
            quaddirect_to_phases<float>({0.5f + back_emf, 0.f}, get_eangle(open_loop_shaft_angle_));
          auto cntr_volts = center_phase_voltages(phase_volts);
          driver_.set_phase_voltages(cntr_volts);
        }
        return;

      case ControllerMode::TORQUE:
        {

          // Convert requested torque into a current request
          float requested_current = target_ / motor_.kT;

          // Let's limit to the stall current of the motor
          // We don't know user's intentions so we can't just limit to safe current
          requested_current =
            std::clamp(requested_current, -motor_.MAX_CURRENT, motor_.MAX_CURRENT);

          // Generate desired current in QD frame, D = 0;
          QuadDirectValues<float> desr_current{requested_current, 0};


          // Pump controllers f
          auto ff_volts = feedforward(desr_current);
          // auto fb_volts = feedback(desr_current);
          // fb_voltage_.at(i_) = fb_volts;
          // auto bemf_volts = back_emf_decoupler();

          const auto dr_volts = center_phase_voltages(filter_phase_voltages(ff_volts)) +
            PhaseValues<float>{1.f, 1.f, 1.f};

          // desr_voltage_.at(i_) = dr_volts;

          driver_.set_phase_voltages(dr_volts);
          // fb_voltage_.at(i_) = driver_.set_phase_voltages(dr_volts);
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

  Butterworth2 vel_filter_;

  PhaseValues<Butterworth2> applited_voltage_filters_;
  PhaseValues<Butterworth2> feedback_voltage_filters_;

  Butterworth2 bemf_filter_;

  PhaseValues<float> phase_currents_{0.f, 0.f, 0.f};
  QuadDirectValues<float> quaddirect_currents_{0.f, 0.f};

  /// \note: Feedback controller variables
  float Kp_;
  float Ki_;

  QuadDirectValues<float> last_error_{0.f, 0.f};
  QuadDirectValues<float> last_command_{0.f, 0.f};

  /// \note: Feedforward controller variables
  PhaseValues<float> last_desr_phase_currents_{0.f, 0.f, 0.f};
  PhaseValues<float> last_desr_phase_voltages_{0.f, 0.f, 0.f};


  ControllerMode ctrl_mode_ = ControllerMode::DISABLE;

  TeensyTimerTool::PeriodicTimer ctrl_timer_;
  TeensyTimerTool::PeriodicTimer print_timer_;

  int control_period_us_ = 100; //us
  float control_period_s_ = 100.f * 1e-6f; // s
  float control_freq_hz_ = 10000.f;

  float filter_cutoff_freq_hz_ = 2500.f;
  float filter_cutoff_freq_hz_current_ = 2500.f;
  float filter_cutoff_freq_hz_vel_ = 100.f;

  Butterworth2 pos_filter_;

  int pos_sensor_dir_ = 1;
  Angle shaft_angle_{0, 0.f};
  int loops_since_tick_ = 0;
  float shaft_velocity_ = 0.f; // rad /s
  float last_vel_angle_ = 0.f;

  float last_shaft_angle_ = 0.f;
  

  float open_loop_shaft_angle_ = 0.f;
  float open_loop_shaft_velocity_ = 0.f;

  float e_ang_offset_ = 0.f; //rad = How much to subtract from mech ang to align w/ electical angle

  float MAX_VOLT_ = 3.f;

  float target_; // Units depend on control mode, either rad /s or Nm

  bool debug_print_ = true;

  const size_t max_size = 10000;

  std::vector<float> time_ = std::vector<float>(max_size, 0.f);
  std::vector<float> ref_current_ = std::vector<float>(max_size, 0.f);
  std::vector<QuadDirectValues<float>> meas_current_ = std::vector<QuadDirectValues<float>>(
    max_size, {0.f, 0.f});
  std::vector<PhaseValues<float>> desr_voltage_ = std::vector<PhaseValues<float>>(
    max_size, {0.f,
      0.f, 0.f});
  std::vector<PhaseValues<float>> fb_voltage_ =
    std::vector<PhaseValues<float>>(max_size, {0.f, 0.f, 0.f});

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
    // // float w = afreq(now);
    // float req_curr = 0.0f;
    // target_ = motor_.kT * req_curr;
    // time_.at(i_) = now;
    // ref_current_.at(i_) = req_curr;
    update_sensors();
    update_control();
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
      Serial.print(fb_voltage_.at(j).a, 6);
      Serial.print("\t");
      Serial.print(fb_voltage_.at(j).b, 6);
      Serial.print("\t");
      Serial.print(fb_voltage_.at(j).c, 6);
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

    /// \note, should maybe change to lag controller?
    float gain_1 = Kp_ + 0.5 * Ki_ * control_period_s_;
    float gain_2 = -Kp_ + 0.5 * Ki_ * control_period_s_;

    auto e_ang = get_eangle(last_shaft_angle_);

    QuadDirectValues<float> error = desr_current - quaddirect_currents_;

    QuadDirectValues<float> new_command = last_command_ + gain_1 * error + gain_2 * last_error_;

    last_command_ = new_command;
    last_error_ = error;

    return quaddirect_to_phases<float>(new_command, e_ang);
  }

  PhaseValues<float> feedforward(QuadDirectValues<float> desr_current)
  {
    PhaseValues<float> desr_phase_currents =
      quaddirect_to_phases<float>(desr_current, get_eangle(last_shaft_angle_));

    float A = 2.f * motor_.phase_L / control_period_s_ + motor_.phase_R;
    float B = 2.f * motor_.phase_L / control_period_s_ - motor_.phase_R;

    PhaseValues<float> desr_phase_voltages = A * desr_phase_currents - B *
      last_desr_phase_currents_ - last_desr_phase_voltages_;

    last_desr_phase_voltages_ = desr_phase_voltages;
    last_desr_phase_currents_ = desr_phase_currents;
    return desr_phase_voltages;
  }

  PhaseValues<float> back_emf_decoupler() const
  {
    QuadDirectValues<float> back_emf{quaddirect_currents_.d, -quaddirect_currents_.q};
    auto decoupler = quaddirect_to_phases<float>(
      shaft_velocity_ * static_cast<float>(motor_.pole_pairs) * motor_.phase_L * back_emf, get_eangle(
        last_shaft_angle_));
        
    auto bemf = quaddirect_to_phases<float>({0.5f * bemf_filter_.filter(motor_.kV * shaft_velocity_), 0.f} , get_eangle(last_shaft_angle_));
    return bemf + decoupler;

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
    return {applited_voltage_filters_.a.filter(phase_volts.a), applited_voltage_filters_.b.filter(
        phase_volts.b), applited_voltage_filters_.c.filter(phase_volts.c)};
  }
  PhaseValues<float> filter_feedback_voltages(PhaseValues<float> phase_volts)
  {
    return {applited_voltage_filters_.a.filter(phase_volts.a), applited_voltage_filters_.b.filter(
        phase_volts.b), applited_voltage_filters_.c.filter(phase_volts.c)};
  }



};

#endif
