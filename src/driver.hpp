#ifndef DRIVER_HPP
#define DRIVER_HPP
#include <Arduino.h>
#include "transformations.hpp"

class Driver
{
public:
  Driver() = default;
  ~Driver() = default;

  Driver(
    const PhaseValues<int> pins, int enable, float PWM_freq = 200000.f, int PWM_res = 8,
    float driver_volts = 24.f, float max_voltage = 24.f)
  : pins_(pins),
    enable_pin_(enable),
    PWM_freq_(PWM_freq),
    PWM_resolution_(PWM_res),
    MAX_PWM_(1 << PWM_res),
    driver_voltage_(driver_volts),
    MAX_VOLT_(max_voltage)
  {}

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

    // Serial.println(MAX_PWM_);

    set_phase_voltages({0.f, 0.f, 0.f});

    disable();
    inited_ = true;
    return inited_;
  }

  void enable()
  {
    digitalWriteFast(enable_pin_, HIGH);
    enabled_ = true;
  }

  void disable()
  {
    set_phase_voltages({0.f, 0.f, 0.f});
    digitalWriteFast(enable_pin_, LOW);
    enabled_ = false;
  }

  bool set_phase_voltages(PhaseValues<float> voltages) const
  {
    if (enabled_) {
      analogWrite(pins_.a, volts_to_PWM(voltages.a));
      analogWrite(pins_.b, volts_to_PWM(voltages.b));
      analogWrite(pins_.c, volts_to_PWM(voltages.c));
    }
    return true;
  }

private:
  const PhaseValues<int> pins_;

  const int enable_pin_;

  bool enabled_ = false;
  bool inited_ = false;

  const float PWM_freq_;
  const int PWM_resolution_;
  const int MAX_PWM_;

  const float driver_voltage_;
  const float MAX_VOLT_;

  int volts_to_PWM(float volt) const
  {
    auto v =
      std::clamp<float>(volt, 0, MAX_VOLT_) / driver_voltage_ * static_cast<float>(MAX_PWM_ - 1);
    return static_cast<int>(v);
  }

};

#endif
