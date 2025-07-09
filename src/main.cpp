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

constexpr float CURR_GAIN = 5.f; // Amps / Volt

InlineCurrentSensor Current_Phase_B{A8, CURR_GAIN};
InlineCurrentSensor Current_Phase_C{A9, CURR_GAIN};
InlineCurrentSensorPackage Current_Sensors{{&Current_Phase_C, &Current_Phase_B}};

constexpr float PWM_FREQ = 20000.f;
constexpr int PWM_RES = 12;

Driver GateDriver{{2, 3, 4}, 1, PWM_FREQ, PWM_RES};

SPIEncoder Encoder{SPI, 10};

MotorParameters EC45_Flat{8, 0.272, 111.f * 1e-6, 3.f, 8.f, 0.034, 0.0369};

MotorController controller_{EC45_Flat, GateDriver, Current_Sensors, Encoder};

// constexpr float phase_resistance_ = 1.5f * 0.26; //Ohm
// constexpr float phase_inductance_ = 1.5f * 111.f * 1e-6f; //Henry

// const float MAX_VOLT = 2.f;

float zero_ang_ = 0.f;
float kappa = 3.f;
float c = 0.01;

void spring()
{
  float diff = zero_ang_ - controller_.get_shaft_angle();
  Serial.println(diff,6);
  if(abs(diff) < 0.01) {
    diff = 0;
  }
  float torque = kappa * diff;
  controller_.set_target(std::clamp(torque, -0.1f, 0.1f));
}

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

  zero_ang_ = controller_.get_shaft_angle();

  // float force = 2.f;
  // constexpr float radius = 42.f / 1000.f * 0.5f;
  controller_.set_target(0.f);

  controller_.start_control(100);
  // timer_.begin(spring, 100);
  // controller_.start_print(10);

  // controller_.start_control(100);
  // Serial.println("=====");

  // start_time_ = micros() * 1e-6;

  // timer_.begin(read_and_run, sampling_period_us_);

}

void loop()
{
  Serial.print(Encoder.read(),6);
  Serial.print("\t");
  Serial.println(controller_.get_shaft_angle(), 6);
  delay(100);
}
