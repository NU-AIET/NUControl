#ifndef DRIVER_HPP
#define DRIVER_HPP
#include <Arduino.h>
#include "transformations.hpp"


/// @brief Three phase motor driver
class BrushlessDriver
{
public:
  BrushlessDriver() = default;
  ~BrushlessDriver() = default;

  /// @brief
  /// @param pins - The PWM pins corresponding to each phase (A, B, C)
  /// @param enable - The digital pin corresponing to the enable
  /// @param PWM_freq - The PWM frequency
  /// @param PWM_res - The PWM resolution (# of bits)
  /// @param driver_volts - The high voltage side of the driver
  /// @param max_voltage - Imposed limit lower than high side voltage
  BrushlessDriver(
    const PhaseValues<int> pins, int enable, float PWM_freq = 200000.f, int PWM_res = 8,
    float driver_volts = 24.f, float max_voltage = 24.f)
  : pins_(pins),
    enable_pin_(enable),
    PWM_freq_(PWM_freq),
    PWM_resolution_(PWM_res),
    MAX_PWM_(1 << PWM_res),
    driver_voltage_(driver_volts),
    MAX_VOLT_(min(driver_volts, max_voltage))
  {}

  /// @brief set all pins for output as well as setup for the PWM frequency and resolution
  /// @returns true if initialization was completed
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

  /// \brief enable the driver
  void enable()
  {
    digitalWriteFast(enable_pin_, HIGH);
    enabled_ = true;
  }

  /// \brief disable the driver, will set phase voltages to 0 as well.
  void disable()
  {
    set_phase_voltages({0.f, 0.f, 0.f});
    digitalWriteFast(enable_pin_, LOW);
    enabled_ = false;
  }

  /// \brief set the phase voltages of the motor
  /// \param voltages - The voltages to be applied to the end of each phase (A, B, C)
  /// \returns the PWM duty in bits applied to each phase
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
  const PhaseValues<int> pins_;

  const int enable_pin_;

  bool enabled_ = false;
  bool inited_ = false;

  const float PWM_freq_;
  const int PWM_resolution_;
  const int MAX_PWM_;

  const float driver_voltage_;
  const float MAX_VOLT_;

  /// @brief converts between volts and PWM duty cycle
  /// @param volts to be applied
  /// @returns the duty cycle in PWM ticks
  int volts_to_PWM(float volt) const
  {
    int v =
      static_cast<int>(std::round(std::clamp<float>(
        volt, 0,
        MAX_VOLT_) / driver_voltage_ * static_cast<float>(MAX_PWM_ - 1)));
    return v;
  }

};

class BrushedDriver{

  public:
    BrushedDriver() = default;
    ~BrushedDriver() = default;

    BrushedDriver(const std::pair<int, int> pins, const int pin_b, int enable, float PWM_freq = 200000.f, int PWM_res = 8,
    float driver_volts = 24.f, float max_voltage = 24.f)
    : pins_(pins_),
    enable_pin_(enable),
    PWM_freq_(PWM_freq),
    PWM_resolution_(PWM_res),
    MAX_PWM_(1 << PWM_res),
    driver_voltage_(driver_volts),
    MAX_VOLT_(min(driver_volts, max_voltage))
  {}

    /// @brief set all pins for output as well as setup for the PWM frequency and resolution
    /// @returns true if initialization was completed
    bool init()
    {
      pinMode(pins_.first, OUTPUT);
      pinMode(pins_.second, OUTPUT);

      pinMode(enable_pin_, OUTPUT);

      analogWriteFrequency(pins_.first, PWM_freq_);
      analogWriteFrequency(pins_.second, PWM_freq_);

      analogWriteResolution(PWM_resolution_);

      // Ensure everything is at 0
      set_phase_voltages({0.f, 0.f});

      disable();
      inited_ = true;
      return inited_;
    }

    /// \brief enable the driver
    void enable()
    {
      digitalWriteFast(enable_pin_, HIGH);
      enabled_ = true;
    }

    /// \brief disable the driver, will set phase voltages to 0 as well.
    void disable()
    {
      set_phase_voltages({0.f, 0.f});
      digitalWriteFast(enable_pin_, LOW);
      enabled_ = false;
    }

    /// \brief set the phase voltages of the motor
    /// \param voltages - The voltages to be applied to the end of each phase (A, B, C)
    /// \returns the PWM duty in bits applied to each phase
    std::pair<float, float> set_phase_voltages(std::pair<float, float> volts) const
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
    bool inited_ = false;

    const float PWM_freq_;
    const int PWM_resolution_;
    const int MAX_PWM_;

    const float driver_voltage_;
    const float MAX_VOLT_;

    /// @brief converts between volts and PWM duty cycle
    /// @param volts to be applied
    /// @returns the duty cycle in PWM ticks
    int volts_to_PWM(float volt) const
    {
      int v =
        static_cast<int>(std::round(std::clamp<float>(
          volt, 0,
          MAX_VOLT_) / driver_voltage_ * static_cast<float>(MAX_PWM_ - 1)));
      return v;
    }


};

#endif
