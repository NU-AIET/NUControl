#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include "filter.hpp"
#include "current_sense.hpp"
#include "driver.hpp"
#include "transformations.hpp"
#include "motor_controller.hpp"

TeensyTimerTool::PeriodicTimer timer_;

constexpr size_t max_size = 25000;

/// @brief Data storage vectors

std::vector<float> time_(max_size, 0);
std::vector<float> applied_voltage_(max_size, 0);
std::vector<float> meas_current_(max_size, 0);
std::vector<float> ref_current_(max_size, 0);

constexpr float CURR_GAIN = 5.f; // Amps / Volt

InlineCurrentSensor Current_Phase_B{A8, CURR_GAIN};
InlineCurrentSensor Current_Phase_C{A9, CURR_GAIN};
InlineCurrentSensorPackage Current_Sensors{{&Current_Phase_C, &Current_Phase_B}};

float read_1 = 0.0f;
float read_2 = 0.0f;

constexpr float PWM_FREQ = 20000.f;
constexpr int PWM_RES = 12;

Driver GateDriver{{2, 3, 4}, 1, PWM_FREQ, PWM_RES};

SPIEncoder Encoder{SPI, 10};

MotorParameters EC45_Flat{8, 2.f * 0.272, 2.f * 111.f * 1e-6, 3.f, 8.f, 0.034, 0.0369};

MotorController controller_{EC45_Flat, GateDriver, Current_Sensors, Encoder};

size_t i = 0;

constexpr int sampling_period_us_ = 100;
constexpr float sampling_period_s_ = static_cast<float>(sampling_period_us_) * 1e-6f;

float max_angular_freq_ = 2.f * PI * 70.f;

float freq_gain = 10.f;
float freq_offset = 1;

constexpr float seconds_per_full_loop_ = static_cast<float>(max_size) * sampling_period_s_;

int loops = 0;
int terminal_loop = 15;

float start_time_ = 0.f;

/// @brief  Converts time to angular frequency
/// @param t - Time in seconds
/// @returns the angular frequency in rad / s
float freq(float t)
{
  return max_angular_freq_;
}

void read_and_run();

constexpr float phase_resistance_ = 1.5f * 0.26; //Ohm
constexpr float phase_inductance_ = 1.5f * 111.f * 1e-6f; //Henry

float last_feedforward = 0.f;
float last_ref = 0.f;

const float MAX_VOLT = 2.f;
void restart_and_run()
{
  GateDriver.enable();
  start_time_ = micros() * 1e-6;
  timer_.begin(read_and_run, sampling_period_us_);

}

void clean_and_close()
{
  timer_.end();
  GateDriver.disable();
  Serial.println("=====");
  for (size_t j = 0; j < i; ++j) {
    Serial.print(time_.at(j), 6);
    Serial.print("\t");
    Serial.print(applied_voltage_.at(j), 6);
    Serial.print("\t");
    Serial.print(meas_current_.at(j), 6);
    Serial.print("\t");
    Serial.println(ref_current_.at(j), 6);
    Serial.flush();
  }
  i = 0;
  delay(1);
  Serial.println("=====");
  Serial.flush();
  delay(10);
  // freq_offset += seconds_per_full_loop_ * freq_gain;
  if (loops < terminal_loop) {
    restart_and_run();
    loops++;
  } else {
    // Serial.println("=====");
    // Serial.flush();
    delay(10);
    exit(1);
  }

}

void read_and_run() {}

void setup()
{
  while (!Serial) {}

  Serial.println("Hell yeah!");

  auto ret_init = controller_.init_components();
  if (!ret_init) {
    Serial.println("Motor controller component failed to init");
    exit(0);
  }

  Serial.println("Aligning");

  auto ret_align = controller_.align_sensors(true);
  if (!ret_align) {
    Serial.println("Motor controller component failed to align");
    exit(0);
  }

  Serial.println("Preparing to run");
  delay(1000);

  controller_.set_control_mode(ControllerMode::TORQUE);

  float force = 2.f;
  constexpr float radius = 42.f / 1000.f * 0.5f;
  controller_.set_target(-force *  radius);

  controller_.start_control(100);
  controller_.start_print(10);

  // controller_.start_control(100);
  // Serial.println("=====");

  // start_time_ = micros() * 1e-6;

  // timer_.begin(read_and_run, sampling_period_us_);

}

void loop()
{
}
