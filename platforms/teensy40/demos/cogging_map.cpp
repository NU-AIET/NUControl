// Runs the anticogging characterization routine on a single motor (EC45_Flat).
// The mapper steps through 200 equally-spaced positions in both directions,
// averaging the holding torque and phase voltages at each point.
// Output is printed as a tab-separated table to Serial when complete; paste it
// into anticog_helpers.hpp to build a lookup table for anticogging compensation.

#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include "nu_control.hpp"

constexpr size_t MAP_STEPS = 200;

TeensyTimerTool::PeriodicTimer timer_(TeensyTimerTool::TCK);

constexpr float CURR_GAIN = 5.f;
constexpr int   ADC_RES   = 10;

InlineCurrentSensor Current_Phase_0{A0, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_1{A1, CURR_GAIN, ADC_RES};
InlineCurrentSensorPackage Current_Sensors1{{&Current_Phase_0, &Current_Phase_1}};

constexpr float PWM_FREQ      = 20000.f;
constexpr int   PWM_RES       = 12;
constexpr float DRIVER_VOLTAGE = 24.f;

const uint16_t EncoderReadCmd = (0b11 << 14) | 0x3FFF;
SPIEncoder      Encoder1{EncoderReadCmd, SPI, 10};
BrushlessDriver GateDriver1{{3, 4, 5}, 2, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};

BrushlessController controller_1{EC45_Flat, GateDriver1, Current_Sensors1, Encoder1,
  [](int ms){ delay(ms); },
  [](const std::string & s){ Serial.println(s.c_str()); }};

CoggingMapper<MAP_STEPS> mapper{controller_1,
  [](int ms){ delay(ms); },
  [](const std::string & s){ Serial.println(s.c_str()); }};

void update()
{
  if (mapper.is_done()) {
    timer_.stop();
    return;
  }
  mapper.step();
}

void setup()
{
  while (!Serial) {}

  TeensyTimerTool::attachErrFunc(timer_errors);
  analogReadAveraging(1);

  Serial.println("Cogging mapper starting.");

  if (!controller_1.init_components()) {
    Serial.println("Motor controller component failed to init");
    exit(0);
  }

  Serial.println("Aligning");

  if (!controller_1.align_sensors()) {
    Serial.println("Motor controller component failed to align");
    exit(0);
  }

  Serial.println("Starting cogging map — do not disturb the motor.");

  // just do 1 loop; up this to more if you want better accuracy
  mapper.map_cogging(1);
  timer_.begin(update, 100);
}

void loop() {}
