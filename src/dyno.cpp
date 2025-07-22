// #include <Arduino.h>
// #include <TeensyTimerTool.h>
// #include <Encoder.h>
// #include "math.h"
// #include "filter.hpp"
// #include <deque>
// #include <numeric>
// #include "driver.hpp"
// #include "motor_controller.hpp"


// MotorParameters Pittman{0, 0.f, 0.f, 0.f, 0.f, 0.f, 1.14974f};

// BrushedDriver DC_driver{std::pair<int, int>(2,3), 1};

// BrushedController controller_{Pittman, DC_driver};

// TeensyTimerTool::PeriodicTimer timer;
// TeensyTimerTool::PeriodicTimer print_timer;


// Encoder DynoEnc(4, 5);

// float get_position()
// {
//     return _2_PI_ * static_cast<float>(DynoEnc.read()) / (2000.f * 19.7f);
// }


// const int update_period_us_ = 100;
// const float update_period_s = update_period_us_ * 1e-6f;

// float last_velocity_ = 0.f;
// float last_read_ = 0.f;

// float velocity = 0.f;
// float pos = 0.f;

// float alpha = 0.25;
// int i = 0;

// Butterworth2 vel_filt{100.f, 1 / update_period_s};

// float update_velocity(float new_sample)
// {
//     float vel =  1.f / (update_period_s) * (new_sample - last_read_);

//     last_read_ = new_sample;

//     return vel_filt.filter(vel);
// }

// void print(){
//     // Serial.print("Position: ");
//     // Serial.print(pos);
//     // Serial.print("\t");
//     Serial.print("Velocity: ");
//     Serial.println(velocity);
// }

// void update()
// {
//     pos = get_position();
//     velocity = update_velocity(pos);
//     // controller_.update_control();
//     // analogWrite(2, 128);
//     // analogWrite(3, 1);

// }

// void setup(){
//     while(!Serial){}

//     // DC_driver.init();
//     // DC_driver.enable();
//     pinMode(2, OUTPUT);
//     pinMode(3, OUTPUT);

//     pinMode(1, OUTPUT);

//     digitalWrite(1, HIGH);
//     digitalWrite(2, HIGH);
//     digitalWrite(3, LOW);

//     controller_.set_control_mode(ControllerMode::OPEN_LOOP_VELOCITY);
//     controller_.set_target(_2_PI_);
//     timer.begin(update, update_period_us_);
//     print_timer.begin(print, 100000);


// }

// void loop(){

// }
