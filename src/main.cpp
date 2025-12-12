#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <vector>
#include <math.h>
#include "nu_control.hpp"
#include <numeric>
#include "anticogging_mapper.hpp"

TeensyTimerTool::PeriodicTimer timer_(TeensyTimerTool::TMR4);

constexpr float CURR_GAIN = 5.f; // Amps / Volt

constexpr int ADC_RES = 10;

InlineCurrentSensor CS1_1{A8, CURR_GAIN, ADC_RES};
InlineCurrentSensor CS1_2{A9, CURR_GAIN, ADC_RES};

InlineCurrentSensor CS2_1{A0, CURR_GAIN, ADC_RES};
InlineCurrentSensor CS2_2{A1, CURR_GAIN, ADC_RES};

InlineCurrentSensorPackage CS1{{&CS1_1, &CS1_2}};
InlineCurrentSensorPackage CS2{{&CS2_1, &CS2_2}};

constexpr float PWM_FREQ = 20000.f;
constexpr int PWM_RES = 12;
constexpr float DRIVER_VOLTAGE = 24.f;

BrushlessDriver Driver1{{3, 4, 5}, 2, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};
BrushlessDriver Driver2{{7, 8, 9}, 6, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};

uint16_t EncoderReadCmd = (0b11 << 14) | 0x3FFF;

SPIEncoder Encoder1{EncoderReadCmd, SPI, 10};
SPIEncoder Encoder2{EncoderReadCmd, SPI1, 0};

BrushlessController MosracController{U2535, Driver1, CS1, Encoder1};
BrushlessController MaxonController{EC45_Flat, Driver2, CS2, Encoder2};

CoggingMapper cogging_mapper{MosracController};


const std::vector<float> MosracCoggingMap
{
  #include "anticogging_map.csv"
};

const PhaseValues<std::vector<float>> MosracVoltageCoggingMap
{
  {
    #include "anticogging_map_va.csv"
    },
  {
    #include "anticogging_map_vb.csv"
    },
  {
    #include "anticogging_map_vc.csv"
    }
};


// int i = 0;

float mosrac_zero_angle = 0.f;
float maxon_zero_angle = 0.f;
float kappa = 1.f;
float charlie = 0.07f;

// K = 5, C = 0.11
// K = 7, C = 0.07

Butterworth2nd<float> pos_filter = Butterworth2nd<float>(5000.f, 10000.f);
Butterworth2nd<float> vel_filter = Butterworth2nd<float>(1000.f, 10000.f);
Butterworth2nd<float> spring_filter = Butterworth2nd<float>(1000.f, 10000.f);
Butterworth2nd<float> damper_filter = Butterworth2nd<float>(10.f, 10000.f);

float MAX_CURRENT = 5.f;

float MAX_TORQUE = min(U2535.kT, EC45_Flat.kT) * MAX_CURRENT;

float last_torque = 0.f;

int indexer = 0;

float last_maxon_read_ = 0.f;
float last_mosrac_read_ = 0.f;

float last_delta_read_ = 0.f;

void bilateral(){
    MosracController.update_sensors();
    MaxonController.update_sensors();

    float mosrac_ang = (MosracController.get_shaft_angle() - mosrac_zero_angle);
    float maxon_ang = (MaxonController.get_shaft_angle() - maxon_zero_angle);

    // last_mosrac_read_ = last_mosrac_read_ * 0.5f + 0.5f * mosrac_ang;
    // last_maxon_read_ = last_maxon_read_ * 0.5f + 0.5f * maxon_ang;

    float delta_pos = (maxon_ang - mosrac_ang);
    float delta_dot = (MaxonController.get_shaft_velocity() - MosracController.get_shaft_velocity());

    if(fabs(delta_pos) > 5){
      timer_.stop();
    }

    // if(fabs(delta_pos) > 0.01f)
    // {
    //   delta_dot = 0.f;
    // }

    float torque = (kappa * delta_pos + charlie * delta_dot);

    // last_delta_read_ = delta_pos;
    // if(fabs(torque) < 0.02f){
    //   torque = 0.f;
    // }

    if(indexer == 1000) {
      Serial.print(mosrac_ang,6);
      Serial.print("\t");
      Serial.print(maxon_ang,6);
      Serial.print("\t");
      Serial.print(delta_dot, 6);
      Serial.print("\t");
      Serial.println(torque, 6);
      indexer = 0;
    }



    float mosrac_torque = std::clamp(torque, -MAX_TORQUE, MAX_TORQUE);
    float maxon_torque = std::clamp(-torque, -MAX_TORQUE, MAX_TORQUE);

    MosracController.set_target(mosrac_torque);
    MaxonController.set_target(maxon_torque);

    MosracController.update_control();
    MaxonController.update_control();
    last_torque = torque;
    ++indexer;
}

void update()
{
  if(indexer == 0) {
    Serial.println("=====");
  }
  auto val_1 = Encoder1.read_raw();
  auto val_2 = Encoder2.read_raw();
  
  Serial.print(val_1);
  Serial.print("\t");
  Serial.println(val_2);

  if(indexer == 10000){
    Serial.println("=====");
    timer_.stop();

  }
  indexer++;
}

void setup()
{
  while (!Serial) {}

  TeensyTimerTool::attachErrFunc(timer_errors);
  analogReadAveraging(1);

  Serial.println("Hell yeah!");


  // auto ret_init = MosracController.init_components();
  // if (!ret_init) {
  //   Serial.println("Motor controller component failed to init");
  //   exit(0);
  // }
  // Serial.println("Aligning");

  // auto ret_align = MosracController.align_sensors(1, false);
  // if (!ret_align) {
  //   Serial.println("Motor controller component failed to align");
  //   exit(0);
  // }

  // ret_init = MaxonController.init_components();
  // if (!ret_init) {
  //   Serial.println("Motor controller component failed to init");
  //   exit(0);
  // }
  // Serial.println("Aligning");

  // ret_align = MaxonController.align_sensors(-1, false);
  // if (!ret_align) {
  //   Serial.println("Motor controller component failed to align");
  //   exit(0);
  // }

  // Serial.println("Preparing to run");
  // delay(1000);

  // MosracController.set_control_mode(ControllerMode::TORQUE);
  // MosracController.set_target(0.f);

  // MaxonController.set_control_mode(ControllerMode::TORQUE);
  // MaxonController.set_target(0.f);

  // // MosracController.enable_anticog(MosracVoltageCoggingMap);

  // MosracController.update_sensors();
  // MaxonController.update_sensors();

  // mosrac_zero_angle = MosracController.get_shaft_angle();
  // maxon_zero_angle = MaxonController.get_shaft_angle();

  // MosracController.start_control(100, false);
  // MaxonController.start_control(100, false);

  // cogging_mapper.map_cogging(1);

  timer_.begin(update, 100);
  // timer_.begin(bilateral, 100);

}

void loop()
{
  // Serial.println(Encoder1.read_raw());
  // Serial.println(Encoder2.read_raw());
  // delay(10);
}
