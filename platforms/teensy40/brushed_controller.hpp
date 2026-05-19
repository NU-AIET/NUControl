// class BrushedController
// {
// public:
//   BrushedController() = default;
//   ~BrushedController() = default;

//   BrushedController(MotorParameters motor, BrushedDriver & motor_driver)
//   : motor_(motor),
//     driver_(motor_driver),
//     ctrl_timer_(TeensyTimerTool::TMR4),
//     print_timer_(TeensyTimerTool::TCK)
//   {}


//   void set_control_mode(ControllerMode mode)
//   {
//     control_mode_ = mode;
//   }

//   void set_target(float target)
//   {
//     target_ = target;
//   }

//   void update_control()
//   {
//     switch (control_mode_) {
//       case ControllerMode::OPEN_LOOP_VELOCITY:
//         {
//           auto v_d = motor_.kV * target_;
//           driver_.set_voltage(center_voltage({v_d, 0}));
//           /* code */
//           return;
//         }

//       default:
//         return;
//     }

//   }

// private:
//   MotorParameters motor_;
//   BrushedDriver & driver_;

//   TeensyTimerTool::PeriodicTimer ctrl_timer_;
//   TeensyTimerTool::PeriodicTimer print_timer_;

//   ControllerMode control_mode_ = ControllerMode::DISABLE;

//   float target_ = 0.f;
//   float MAX_VOLTAGE_ = 24.f;

//   std::pair<float, float> center_voltage(std::pair<float, float> volts)
//   {
//     auto min_ = min(volts.first, volts.second);

//     float ratio = 1.f;
//     auto max_ = max(volts.first, volts.second);
//     if (max_ > MAX_VOLTAGE_) {
//       ratio = MAX_VOLTAGE_ / max_;
//     }
//     return {ratio * (volts.first - min_) + 1, ratio * (volts.second - min_) + 1};

//   }


// };