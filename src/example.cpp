// #include <Arduino.h>
// #include <TeensyTimerTool.h>
// #include <vector>
// #include <math.h>
// #include "nu_control.hpp"

// TeensyTimerTool::PeriodicTimer timer_(TeensyTimerTool::TCK);

// constexpr float CURR_GAIN = 5.f; // Amps / Volt

// constexpr int ADC_RES = 10;

// InlineCurrentSensor Current_Phase_B{A8, CURR_GAIN, ADC_RES};
// InlineCurrentSensor Current_Phase_C{A9, CURR_GAIN, ADC_RES};
// InlineCurrentSensorPackage Current_Sensors{{&Current_Phase_C, &Current_Phase_B}};

// constexpr float PWM_FREQ = 20000.f;
// constexpr int PWM_RES = 12;
// constexpr float DRIVER_VOLTAGE = 24.f;

// BrushlessDriver GateDriver{{2, 3, 4}, 1, PWM_FREQ, PWM_RES, DRIVER_VOLTAGE};

// SPIEncoder Encoder{SPI, 10};

// BrushlessController controller_{EC45_Flat, GateDriver, Current_Sensors, Encoder};


// void setup()
// {
//   while (!Serial) {}

//   TeensyTimerTool::attachErrFunc(timer_errors);
//   analogReadAveraging(1);

//   Serial.println("Hell yeah!");

//   auto ret_init = controller_.init_components();
//   if (!ret_init) {
//     Serial.println("Motor controller component failed to init");
//     exit(0);
//   }
//   Serial.println("Aligning");

//   auto ret_align = controller_.align_sensors(true, true);
//   if (!ret_align) {
//     Serial.println("Motor controller component failed to align");
//     exit(0);
//   }

//   controller_.set_e_angle_offset(0.97f);

//   Serial.println("Preparing to run");
//   delay(1000);

//   controller_.set_control_mode(ControllerMode::TORQUE);

//   controller_.set_target(0.f);

//   controller_.start_control(100);

// }

// void loop() {}
