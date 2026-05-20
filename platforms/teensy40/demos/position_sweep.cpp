// Sweeps a single brushless motor (EC45_Flat) through 10 evenly-spaced angular
// targets (0 – 5.4 rad) in a back-and-forth pattern using PD torque control.
// Prints the active target to Serial each time it advances (~1 s per step).

#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include "nu_control.hpp"
#ifdef NU_TELEMETRY
#include "telemetry.hpp"
#endif

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
BrushlessController controller_1{EC45_Flat, GateDriver1, Current_Sensors1, Encoder1,
  [](const std::string & s){ Serial.println(s.c_str()); },
  [](int ms){ delay(ms); }}; // DISTAL

int idx = 0;
size_t cntr = 0;
std::array<float, 10> targets_{0.f, 0.6f, 1.2f, 1.8f, 2.4f, 3.0f, 3.6f, 4.2f, 4.8f, 5.4f};
float kp = 0.1f;
float kd = -0.003f;
static int direction = 1;

#ifdef NU_TELEMETRY
MotorTelemetry telemetry;
constexpr size_t TELEM_PERIOD_TICKS = 10;
#endif

void update(){
  controller_1.update_sensors();

  auto vel_error = controller_1.get_shaft_velocity() - 0.f;
  auto torque = kp * normalize_angle(targets_[idx] - controller_1.get_shaft_angle()) + kd * vel_error;
  torque = std::clamp(torque, -0.3f, 0.3f);

  controller_1.set_target(torque);
  controller_1.update_control();
  if(cntr >= 10000){
    idx += direction;
    cntr = 0;
    if(idx == 10) {
      direction = -1;
      idx = 8;
    }
    if (idx == -1) {
      direction = 1;
      idx = 1;
    }
    Serial.println("Angular position target: " + String(targets_[idx]) + " rad");
  }
  cntr++;

#ifdef NU_TELEMETRY
  if (cntr % TELEM_PERIOD_TICKS == 0) {
    telemetry.populate(controller_1);
    telemetry.serialize(SerialUSB1);
  }
#endif
}

void setup()
{
  while (!Serial) {}
#ifdef NU_TELEMETRY
  SerialUSB1.begin(0);
#endif

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

  Serial.println("RUNNING POSITION SWEEP DEMO");
  delay(1000);

  controller_1.set_control_mode(ControllerMode::TORQUE);
  controller_1.set_position_filter({{0.25f, 0.25f, 0.25f, 0.25f}, {}});
  controller_1.set_target(0.03f);
  controller_1.set_feedback_state(true);
  controller_1.set_back_emf_comp_state(false);

  controller_1.start_control(100);
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
