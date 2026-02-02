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

std::vector<float> vel_coeffs_ = {28.9734326,27.52295677,26.10537177,24.72036642,23.36762954
,22.04684994,20.75771642,19.49991782,18.27314293,17.07708058
,15.91141957,14.77584873,13.67005687,12.5937328,11.54656533
,10.52824328,9.53845547,8.5768907,7.6432378,6.73718557
,5.85842283,5.0066384,4.18152108,3.3827597,2.61004306
,1.86305999,1.14149929,0.44504978,-0.22659973,-0.87376042
,-1.49674347,-2.09586008,-2.67142143,-3.22373871,-3.7531231
,-4.25988579,-4.74433795,-5.20679079,-5.64755549,-6.06694322
,-6.46526518,-6.84283255,-7.19995652,-7.53694828,-7.854119
,-8.15177989,-8.43024211,-8.68981686,-8.93081532,-9.15354869
,-9.35832813,-9.54546485,-9.71527003,-9.86805485,-10.00413049
,-10.12380815,-10.22739901,-10.31521426,-10.38756508,-10.44476266
,-10.48711818,-10.51494283,-10.52854779,-10.52824426,-10.51434341
,-10.48715643,-10.44699452,-10.39416884,-10.3289906,-10.25177098
,-10.16282115,-10.06245232,-9.95097565,-9.82870235,-9.69594359
,-9.55301057,-9.40021446,-9.23786645,-9.06627773,-8.88575949
,-8.69662291,-8.49917917,-8.29373946,-8.08061498,-7.86011689
,-7.6325564,-7.39824468,-7.15749292,-6.91061231,-6.65791403
,-6.39970928,-6.13630922,-5.86802506,-5.59516797,-5.31804915
,-5.03697977,-4.75227103,-4.4642341,-4.17318018,-3.87942046
,-3.58326611,-3.28502832,-2.98501829,-2.68354718,-2.3809262
,-2.07746652,-1.77347934,-1.46927584,-1.16516719,-0.8614646
,-0.55847925,-0.25652231,0.04409502,0.34306155,0.6400661
,0.93479749,1.22694453,1.51619604,1.80224082,2.08476769
,2.36346547,2.63802297,2.908129,3.17347238,3.43374192
,3.68862644,3.93781476,4.18099567,4.41785801,4.64809058
,4.87138219,5.08742167,5.29589782,5.49649947,5.68891542
,5.87283448,6.04794548,6.21393723,6.37049853,6.51731821
,6.65408508,6.78048795,6.89621565,7.00095697,7.09440074
,7.17623576,7.24615086,7.30383485,7.34897655,7.38126476
,7.40038829,7.40603598,7.39789662,7.37565904,7.33901204
,7.28764445,7.22124507,7.13950272,7.04210621,6.92874436
,6.79910599,6.6528799,6.48975491,6.30941984,6.11156349
,5.89587469,5.66204224,5.40975497,5.13870168,4.84857119
,4.53905232,4.20983387,3.86060467,3.49105352,3.10086924
,2.68974065,2.25735655,1.80340577,1.32757711,0.8295594
,0.30904144,-0.23428795,-0.80073995,-1.39062576,-2.00425655
,-2.64194352,-3.30399785,-3.99073073,-4.70245333,-5.43947686
,-6.20211248,-6.9906714,-7.80546479,-8.64680384,-9.51499974
,-10.41036366,-11.33320681,-12.28384036,-13.2625755,-14.26972342};

DiscreteFilter<float, float> vel_filter{vel_coeffs_, {}};

std::vector<float> pos_coeffs_{0.25f, 0.25f, 0.25f, 0.25f};

DiscreteFilter<float, float> pos_filter{pos_coeffs_, {}};

// CoggingMapper cogging_mapper{MosracController};


// const std::vector<float> MosracCoggingMap
// {
//   #include "anticogging_map.csv"
// };

// const PhaseValues<std::vector<float>> MosracVoltageCoggingMap
// {
//   {
//     #include "anticogging_map_va.csv"
//     },
//   {
//     #include "anticogging_map_vb.csv"
//     },
//   {
//     #include "anticogging_map_vc.csv"
//     }
// };


// // int i = 0;

float mosrac_zero_angle = 0.f;
float maxon_zero_angle = 0.f;
// float kappa = 1.f;
// float charlie = 0.07f;

// // K = 5, C = 0.11
// // K = 7, C = 0.07

// Butterworth2nd<float> pos_filter = Butterworth2nd<float>(5000.f, 10000.f);
// Butterworth2nd<float> vel_filter = Butterworth2nd<float>(1000.f, 10000.f);
// Butterworth2nd<float> spring_filter = Butterworth2nd<float>(1000.f, 10000.f);
// Butterworth2nd<float> damper_filter = Butterworth2nd<float>(10.f, 10000.f);

// float MAX_CURRENT = 5.f;

// float MAX_TORQUE = min(U2535.kT, EC45_Flat.kT) * MAX_CURRENT;

// float last_torque = 0.f;

// int indexer = 0;

// float last_maxon_read_ = 0.f;
// float last_mosrac_read_ = 0.f;

// float last_delta_read_ = 0.f;

// void bilateral(){
//     MosracController.update_sensors();
//     MaxonController.update_sensors();

//     float mosrac_ang = (MosracController.get_shaft_angle() - mosrac_zero_angle);
//     float maxon_ang = (MaxonController.get_shaft_angle() - maxon_zero_angle);

//     // last_mosrac_read_ = last_mosrac_read_ * 0.5f + 0.5f * mosrac_ang;
//     // last_maxon_read_ = last_maxon_read_ * 0.5f + 0.5f * maxon_ang;

//     float delta_pos = (maxon_ang - mosrac_ang);
//     float delta_dot = (MaxonController.get_shaft_velocity() - MosracController.get_shaft_velocity());

//     if(fabs(delta_pos) > 5){
//       timer_.stop();
//     }

//     // if(fabs(delta_pos) > 0.01f)
//     // {
//     //   delta_dot = 0.f;
//     // }

//     float torque = (kappa * delta_pos + charlie * delta_dot);

//     // last_delta_read_ = delta_pos;
//     // if(fabs(torque) < 0.02f){
//     //   torque = 0.f;
//     // }

//     if(indexer == 1000) {
//       Serial.print(mosrac_ang,6);
//       Serial.print("\t");
//       Serial.print(maxon_ang,6);
//       Serial.print("\t");
//       Serial.print(delta_dot, 6);
//       Serial.print("\t");
//       Serial.println(torque, 6);
//       indexer = 0;
//     }



//     float mosrac_torque = std::clamp(torque, -MAX_TORQUE, MAX_TORQUE);
//     float maxon_torque = std::clamp(-torque, -MAX_TORQUE, MAX_TORQUE);

//     MosracController.set_target(mosrac_torque);
//     MaxonController.set_target(maxon_torque);

//     MosracController.update_control();
//     MaxonController.update_control();
//     last_torque = torque;
//     ++indexer;
// }

// void update()
// {
//   if(indexer == 0) {
//     Serial.println("=====");
//   }
//   auto val_1 = Encoder1.read_raw();
//   auto val_2 = Encoder2.read_raw();
  
//   Serial.print(val_1);
//   Serial.print("\t");
//   Serial.println(val_2);

//   if(indexer == 10000){
//     Serial.println("=====");
//     timer_.stop();

//   }
//   indexer++;
// }

void pos_cont(){
  MaxonController.update_sensors();
  auto ang = MaxonController.get_shaft_angle();


  auto torque = std::clamp(1.f * (maxon_zero_angle - ang), EC45_Flat.kT * -3.f, EC45_Flat.kT * 3.f);

  Serial.println(torque);

  MaxonController.set_target(torque);
  MaxonController.update_control();

  // indx++;
  // if(indx = 100){
  //   // Serial.print(ang, 6);
  //   // Serial.print("\t");
  //   // Serial.println(MosracController.get_shaft_velocity(), 6);
  //   indx = 0;
  // }

}

void setup()
{
  while (!Serial) {}

//   TeensyTimerTool::attachErrFunc(timer_errors);
//   analogReadAveraging(1);

//   Serial.println("Hell yeah!");


  // if (!MosracController.init_components()) {
  //   Serial.println("Motor controller component failed to init");
  //   exit(0);
  // }
  // Serial.println("Aligning");

  // auto ret_align = 
  // if (!MosracController.align_sensors(1, false)) {
  //   Serial.println("Motor controller component failed to align");
  //   exit(0);
  // }

  if (!MaxonController.init_components()) {
    Serial.println("Motor controller component failed to init");
    exit(0);
  }
  Serial.println("Aligning");

  if (!MaxonController.align_sensors(-1, false)) {
    Serial.println("Motor controller component failed to align");
    exit(0);
  }

  Serial.println("Preparing to run");
  delay(1000);

  // MosracController.set_control_mode(ControllerMode::TORQUE);
  // MosracController.set_target(0.f);

  MaxonController.set_control_mode(ControllerMode::TORQUE);
  MaxonController.set_target(0.f);

  MaxonController.set_feedforward_state(true);
  MaxonController.set_feedback_state(false);
  MaxonController.set_back_emf_comp_state(false);


  // MosracController.enable_anticog(MosracVoltageCoggingMap);

  MaxonController.set_velocity_filter(vel_filter);
  MaxonController.set_position_filter(pos_filter);


  // MosracController.update_sensors();
  MaxonController.update_sensors();

  // mosrac_zero_angle = MosracController.get_shaft_angle();
  maxon_zero_angle = MaxonController.get_shaft_angle();

  // MosracController.start_control(100, false);
  MaxonController.start_control(100, false);

//   // cogging_mapper.map_cogging(1);

  // timer_.begin(update, 100);
  // timer_.begin(bilateral, 100);
  timer_.begin(pos_cont, 100);

}

void loop()
{
  // Serial.println(Encoder1.read_raw());
  // Serial.println(Encoder2.read_raw());
  // delay(10);
}
