#ifndef NUCONTROL_CORE_COGGING_MAPPER_HPP
#define NUCONTROL_CORE_COGGING_MAPPER_HPP

#include <array>
#include <algorithm>
#include <string>
#include <functional>
#include "brushless_controller.hpp"

template<std::size_t steps_>
class CoggingMapper
{
public:
  using LogFn   = std::function<void(const std::string &)>;
  using SleepFn = std::function<void(int /*ms*/)>;

  CoggingMapper() = default;
  ~CoggingMapper() = default;

  CoggingMapper(BrushlessController & controller, SleepFn sleep_fn, LogFn log_fn = [](const std::string &){})
  : controller_(controller),
    sleep_(sleep_fn),
    log_(log_fn)
  {}

  void map_cogging(int loops, bool disable_cogging = true)
  {
    max_loop_ = loops;
    looper_   = 0;
    idx_      = 0;
    done_     = false;
    target_selector(dir_);
    if (disable_cogging) { controller_.disable_anticog(); }
    auto motor = controller_.get_motor();
    max_torque_ = 0.5f * motor.kT * motor.SAFE_CURRENT;
    controller_.start_control(100);
  }

  // Call this at 100 µs intervals (e.g. from a timer ISR or loop).
  void step()
  {
    if (done_) { return; }
    cogging_step();
  }

  bool is_done() const { return done_; }

  void set_timeout_state(bool state, size_t timeout_max = 200000, float timeout_val = -127.f)
  {
    timeout_state_ = state;
    timeout_max_   = timeout_max;
    timeout_val_   = timeout_val;
  }

  void set_controller_gains(float kp, float ki, float kd, float intl_max)
  {
    kP_       = kp;
    kI_       = ki;
    kD_       = kd;
    max_intl_ = intl_max;
  }

  const std::array<float, steps_> & positions() const { return positions_; }
  const std::array<float, steps_> & torques()   const { return torques_; }
  const std::array<PhaseValues<float>, steps_> & phase_volts() const { return phase_volts_; }

private:
  BrushlessController & controller_;
  SleepFn sleep_;
  LogFn   log_;

  std::array<float, steps_>              positions_{};
  std::array<float, steps_>              torques_{};
  std::array<PhaseValues<float>, steps_> phase_volts_{};

  float pos_target_  = 0.f;
  size_t idx_        = 0;

  bool   done_          = false;
  bool   timeout_state_ = true;
  float  timeout_val_   = -127.f;
  size_t timeout_clk_   = 0;
  size_t timeout_max_   = 200000;

  float kP_       = 1.f;
  float kD_       = 0.1f;
  float kI_       = 0.5f;
  float intl_     = 0.f;
  float max_intl_ = 0.25f;
  float max_torque_ = 0.5f;

  int dir_      = 1;
  int max_loop_ = 10;
  int looper_   = 0;

  const float dt_ = 100.f * 1e-6f;

  size_t clk_start_ = 0;
  float  pos_err_tol_ = 0.01f;
  bool   locked_ = false;

  constexpr static size_t torque_steps_ = 1000;
  constexpr static float  step_inv_     = 1.f / static_cast<float>(torque_steps_);
  float           torque_sum_ = 0.f;
  PhaseValues<float> volt_sum_{0.f, 0.f, 0.f};

  void target_selector(int direction = 1)
  {
    float gain   = static_cast<float>(direction) * _2_PI_;
    float offset = 0.f;
    float shift  = 0.f;

    if (direction == -1) {
      offset = _2_PI_;
      shift  = 1.f;
    }

    for (size_t i = 0; i < steps_; ++i) {
      positions_.at(i) = offset + gain * (i + shift) / static_cast<float>(steps_);
      log_(std::to_string(positions_.at(i)));
    }
  }

  void cogging_step()
  {
    controller_.update_sensors();
    float error = normalize_angle(positions_.at(idx_) - controller_.get_shaft_angle());
    intl_ += (error * dt_);
    intl_ = std::clamp(intl_, -max_intl_, max_intl_);
    float torque = kP_ * error - kD_ * controller_.get_shaft_velocity() + kI_ * intl_;
    torque = std::clamp(torque, -max_torque_, max_torque_);
    controller_.set_target(torque);
    controller_.update_control();
    timeout_clk_++;

    if (timeout_state_ && timeout_clk_ >= timeout_max_) {
      timeout_clk_ = 0;
      torques_.at(idx_)    = timeout_val_;
      phase_volts_.at(idx_) = {timeout_val_, timeout_val_, timeout_val_};
      volt_sum_   = 0.f;
      torque_sum_ = 0.f;
      locked_     = false;
      clk_start_  = 0;
      intl_       = 0.f;
      log_("Target Timedout");
      idx_++;
      log_("Target #" + std::to_string(idx_));
    }

    if (fabs(error) < pos_err_tol_ && !locked_) { locked_ = true; }
    if (locked_) {
      if (fabs(error) > pos_err_tol_) {
        locked_     = false;
        volt_sum_   = 0.f;
        torque_sum_ = 0.f;
        clk_start_  = 0;
      }
      torque_sum_ += torque * step_inv_;
      volt_sum_   += controller_.get_last_phasevolts() * step_inv_;
      clk_start_++;
      if (clk_start_ >= torque_steps_) {
        timeout_clk_         = 0;
        torques_.at(idx_)    = torque_sum_;
        phase_volts_.at(idx_) = volt_sum_;
        volt_sum_   = 0.f;
        torque_sum_ = 0.f;
        locked_     = false;
        clk_start_  = 0;
        intl_       = 0.f;
        idx_++;
        log_("Target #" + std::to_string(idx_));
      }
    }

    if (idx_ >= steps_) {
      report_out();
    }
  }

  void report_out()
  {
    controller_.stop_control();
    log_("=====");

    for (size_t j = 0; j < steps_; ++j) {
      log_(std::to_string(positions_.at(j))    + "\t" +
           std::to_string(torques_.at(j))      + "\t" +
           std::to_string(phase_volts_.at(j).a) + "\t" +
           std::to_string(phase_volts_.at(j).b) + "\t" +
           std::to_string(phase_volts_.at(j).c));
      sleep_(1);
    }

    sleep_(1);
    log_("=====");
    sleep_(10);
    log_("Loop #" + std::to_string(looper_) + " finished!");

    looper_++;

    if (looper_ < max_loop_) {
      idx_ = 0;
      controller_.start_control(100);
      return;
    }

    if (dir_ == 1) {
      dir_    = -1;
      looper_ = 0;
      target_selector(dir_);
      idx_ = 0;
      controller_.start_control(100);
      return;
    }

    log_("Finished!");
    done_ = true;
  }
};

#endif // NUCONTROL_CORE_COGGING_MAPPER_HPP
