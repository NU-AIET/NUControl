// Applies a constant torque to a BLDC motor (EC45_Flat) until it exceeds MAX_VEL rad/s
// then stops the motor.
// Prints the current angular velocity + position from the encoder at 1 Hz for debugging.

#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include "nu_control.hpp"

TeensyTimerTool::PeriodicTimer timer_(TeensyTimerTool::TCK);

constexpr float CURR_GAIN = 5.f; // Amps / Volt
constexpr int ADC_RES = 10;

// pins 14 & 15 on the teensy are connected to the ADC for current sensing
InlineCurrentSensor Current_Phase_0{A0, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_1{A1, CURR_GAIN, ADC_RES};
InlineCurrentSensorPackage Current_Sensors1{{&Current_Phase_0, &Current_Phase_1}};

constexpr float PWM_FREQ = 20000.f;
constexpr int PWM_RES = 12;
constexpr float DRIVER_VOLTAGE = 24.f;

const uint16_t EncoderReadCmd = (0b11 << 14) | 0x3FFF;
SPIEncoder Encoder1{EncoderReadCmd, SPI, 10};
BrushlessDriver GateDriver1{{3, 4, 5}, 2, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};
BrushlessController controller_1{EC45_Flat, GateDriver1, Current_Sensors1, Encoder1}; // DISTAL

size_t cntr = 0;

void update(){
  const auto MAX_VEL = 160.f; // rad/s

  controller_1.update_sensors();

  // print shaft velocity at 1Hz for debugging
  if(cntr % 10000 == 0){
    Serial.print("Shaft Angle: ");
    Serial.print(controller_1.get_shaft_angle());
    Serial.print("\tShaft Velocity: ");
    Serial.print(controller_1.get_shaft_velocity());
    Serial.print("\tT Setpoint: ");
    Serial.println(controller_1.get_target());
  }

  // overspeed governor for safety
  if (fabs(controller_1.get_shaft_velocity()) > MAX_VEL) {
    Serial.println("Overspeed! Stopping motor. (vel=" + String(controller_1.get_shaft_velocity()) + ")");
    controller_1.set_target(0.f);
  }
  controller_1.update_control();

  cntr++;
}

void setup()
{
  while (!Serial) {}

  TeensyTimerTool::attachErrFunc(timer_errors);
  analogReadAveraging(1);

  Serial.println("Hello World!");

  if (!controller_1.init_components()) {
    Serial.println("Motor controller component failed to init");
    exit(0);
  }

  Serial.println("Aligning");

  if (!controller_1.align_sensors()) {
    Serial.println("Motor controller component failed to align");
    exit(0);
  }

  Serial.println("RUNNING ONE_SHOT DEMO");
  delay(1000);

  controller_1.set_control_mode(ControllerMode::TORQUE);
  controller_1.set_position_filter({{0.25f, 0.25f, 0.25f, 0.25f}, {}});
  controller_1.set_target(0.03f);
  controller_1.set_feedback_state(true);
  controller_1.set_back_emf_comp_state(false);

  controller_1.start_control(100,false);
  timer_.begin(update, 100);
}

void loop() {
  // Serial.print(Encoder1.read(), 6);
  // Serial.print("\t");
  // // Serial.print(Encoder2.read(), 6);
  // // Serial.print("\t");
  // // Serial.println(Encoder3.read(), 6);
  // delay(10);
}
