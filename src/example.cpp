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

// unnecessary for my demo hardware.
InlineCurrentSensor Current_Phase_2{A2, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_3{A3, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_4{A4, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_5{A5, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_6{A6, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_7{A7, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_8{A9, CURR_GAIN, ADC_RES};
InlineCurrentSensor Current_Phase_9{A8, CURR_GAIN, ADC_RES};
InlineCurrentSensorPackage Current_Sensors2{{&Current_Phase_4, &Current_Phase_5}};
InlineCurrentSensorPackage Current_Sensors3{{&Current_Phase_2, &Current_Phase_3}};

constexpr float PWM_FREQ = 20000.f;
constexpr int PWM_RES = 12;
constexpr float DRIVER_VOLTAGE = 24.f;

const uint16_t EncoderReadCmd = (0b11 << 14) | 0x3FFF;
SPIEncoder Encoder1{EncoderReadCmd, SPI, 10};
SPIEncoder Encoder2{EncoderReadCmd, SPI1, 0};
SPIEncoder Encoder3{EncoderReadCmd, SPI2, 36};

BrushlessDriver GateDriver1{{3, 4, 5}, 2, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};
BrushlessDriver GateDriver2{{9, 8, 7}, 6, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};
BrushlessDriver GateDriver3{{33, 29, 39}, 38, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};

MotorParameters U2523{7, 0.72487f, 510.f * 1e-6f, 3.f, 8.f, 0.025f, 0.001f};

BrushlessController controller_1{EC45_Flat, GateDriver1, Current_Sensors1, Encoder1}; // DISTAL
BrushlessController controller_2{U2535, GateDriver2, Current_Sensors2, Encoder3}; // PROXIMAL
BrushlessController controller_3{U2535, GateDriver3, Current_Sensors3, Encoder1}; // SPLAY
// BrushlessController controller_{EC45_Flat, GateDriver, Current_Sensors, Encoder};

// CoggingMapper<100> mapper_(controller_2);


// const PhaseValues<std::array<float, 100>>exo_splay_volt_map
// {
//   {
//     #include "splay_map_va.csv"
//   },
//   {
//     #include "splay_map_vb.csv"
//   },
//   {
//     #include "splay_map_vc.csv"
//   }};

// const PhaseValues<std::array<float, 100>>exo_proximal_volt_map
// {
//   {
//     #include "anticogging_map_va.csv"
//   },
//   {
//     #include "anticogging_map_vb.csv"
//   },
//   {
//     #include "anticogging_map_vc.csv"
//   }};

// // const std::array<float, 100>

// const PhaseValues<std::array<float, 100>>exo_distal_volt_map
// {
//   {
//     #include "distal_map_va.csv"
//   },
//   {
//     #include "distal_map_vb.csv"
//   },
//   {
//     #include "distal_map_vc.csv"
//   }};

// AnticoggingCompensator<100> splay_anticog_{exo_splay_volt_map};
// AnticoggingCompensator<100> proximal_anticog_{exo_proximal_volt_map};
// AnticoggingCompensator<100> distal_anticog_{exo_distal_volt_map};

// PhaseValues<float> splay_cog(float rads){
//   return splay_anticog_.get_cogging_voltage(rads);
// }

// PhaseValues<float> prox_cog(float rads){
//   return proximal_anticog_.get_cogging_voltage(rads);
// }

// PhaseValues<float> dist_cog(float rads){
//   return distal_anticog_.get_cogging_voltage(rads);
// }

// BrushlessCalibration splay_calib{{-1, 0, 1}, {0, 1, 1}, 1, 1.970533};
// BrushlessCalibration prox_calib{{-1, 0, 1}, {0, 1, 1}, 1, 1.839425};
// BrushlessCalibration dist_calib{{0, -1, 1}, {1, 0, 1}, -1, -0.561399};

size_t idx = 0;
size_t cntr = 0;
std::array<float, 10> targets_{0.f, 0.6f, 1.2f, 1.8f, 2.4f, 3.0f, 3.6f, 4.2f, 4.8f, 5.4f};
float kp = 0.1f;

void update(){
  const auto MAX_VEL = 10.f; // rad/s

  controller_1.update_sensors();

  // overspeed governor for safety
  if (fabs(controller_1.get_shaft_velocity()) > MAX_VEL) {
    controller_1.set_target(0.f);
  }
  controller_1.update_control();

  // controller_2.update_sensors();
  // controller_2.update_control();
  // controller_3.update_sensors();
  // controller_3.update_control();
  // auto torque = kp * normalize_angle(targets_[idx] - controller_1.get_shaft_angle());
  // torque = std::clamp(torque, -0.3f, 0.3f);

  // controller_1.set_target(torque);
  // controller_1.update_control();
  // if(cntr >= 10000){
  //   idx++;
  //   cntr = 0;
  //   if(idx == 10) {
  //     idx = 0;
  //   }
  // }
  // cntr++;

}

void setup()
{
  while (!Serial) {}

  TeensyTimerTool::attachErrFunc(timer_errors);
  analogReadAveraging(1);

  Serial.println("Hell yeah!");

  // if (!controller_.init_components()) {
  //   Serial.println("Motor controller component failed to init");
  //   exit(0);
  // }

  // Serial.println("Aligning");

  // if (!controller_.align_sensors()) {
  //   Serial.println("Motor controller component failed to align");
  //   exit(0);
  // }

  // Serial.println("Preparing to run");
  // controller_.print_calibration();
  // delay(1000);

  // controller_.set_control_mode(ControllerMode::TORQUE);
  // controller_.set_position_filter({{0.25f, 0.25f, 0.25f, 0.25f}, {}});
  // controller_.set_velocity_filter(vel_filter_200_);
  // controller_.set_target(0.f);
  // controller_.set_feedback_state(true);
  // // controller_.enable_anticog(std::function<PhaseValues<float>(float)>(torque_cog));

  // // mapper_.map_cogging(2);

  // controller_.start_control(100,false);
  // timer_.begin(update, 100);


  if (!controller_1.init_components()) {
    Serial.println("Motor controller component failed to init");
    exit(0);
  }

  Serial.println("Aligning");

  if (!controller_1.align_sensors()) {
    Serial.println("Motor controller component failed to align");
    exit(0);
  }

  // if (!controller_2.init_components()) {
  //   Serial.println("Motor controller component failed to init");
  //   exit(0);
  // }

  // Serial.println("Aligning");

  // if (!controller_2.align_sensors()) {
  //   Serial.println("Motor controller component failed to align");
  //   exit(0);
  // }


  // if (!controller_3.init_components()) {
  //   Serial.println("Motor controller component failed to init");
  //   exit(0);
  // }

  // Serial.println("Aligning");

  // if (!controller_3.align_sensors()) {
  //   Serial.println("Motor controller component failed to align");
  //   exit(0);
  // }

  Serial.println("Preparing to run");
  // controller_2.print_calibration();
  delay(1000);

  controller_1.set_control_mode(ControllerMode::TORQUE);
  controller_1.set_position_filter({{0.25f, 0.25f, 0.25f, 0.25f}, {}});
  controller_1.set_velocity_filter(vel_filter_200_);
  controller_1.set_target(0.03f);
  controller_1.set_feedback_state(true);
  controller_1.set_back_emf_comp_state(false);
  // controller_1.enable_anticog(std::function<PhaseValues<float>(float)>(dist_cog));

  // controller_2.set_control_mode(ControllerMode::TORQUE);
  // controller_2.set_position_filter({{0.25f, 0.25f, 0.25f, 0.25f}, {}});
  // controller_2.set_velocity_filter(vel_filter_200_);
  // controller_2.set_target(0.f);
  // controller_2.set_feedback_state(true);
  // controller_2.enable_anticog(std::function<PhaseValues<float>(float)>(prox_cog));

  // controller_3.set_control_mode(ControllerMode::TORQUE);
  // controller_3.set_position_filter({{0.25f, 0.25f, 0.25f, 0.25f}, {}});
  // controller_3.set_velocity_filter(vel_filter_200_);
  // controller_3.set_target(0.f);
  // controller_3.set_feedback_state(false);
  // controller_3.enable_anticog(std::function<PhaseValues<float>(float)>(splay_cog));

  // mapper_.map_cogging(1);

  controller_1.start_control(100,false);
  // controller_2.start_control(100,false);
  // controller_3.start_control(100,false);
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
