#include "motor_controller.hpp"

class CoggingMapper
{

    public:
        CoggingMapper() = default;
        ~CoggingMapper() = default;

        CoggingMapper(BrushlessController & controller)
        : controller_(controller),
          timer_(TeensyTimerTool::TCK)
        {};

        void map_cogging(int loops)
        {
            max_loop = loops;
            looper = 0;
            idx = 0;
            target_selector(dir);
            controller_.start_control(100, false);
            timer_.begin([this] {cogging_mapper();}, 100);
        }

    private:

        BrushlessController & controller_;
        TeensyTimerTool::PeriodicTimer timer_;

        // size_t COGGING_STEPS = 1000;

        std::vector<float> positions_ = std::vector<float>(COGGING_STEPS, 0.f);
        std::vector<float> torques_ = std::vector<float>(COGGING_STEPS, 0.f);
        std::vector<PhaseValues<float>> phase_volts_ = std::vector<PhaseValues<float>>(COGGING_STEPS, {0.f, 0.f, 0.f});

        float pos_target = 0.f;
        size_t idx = 0;

        // Controller parameters

        float kP = 2.5f;
        float kD = 0.001;
        float kI = 4.f;
        float intl = 0.f;

        int dir =  1;
        int max_loop = 10;
        int looper = 0;

        int clk_start = 0;
        float pos_err_tol = 0.0025f;
        bool locked = false;
        size_t torque_steps_ = 1000; // Control Frequency * torque_steps_ = settling time = 0.1s by default
        std::vector<float> temp_torques_ = std::vector<float>(torque_steps_, 0.f);
        std::vector<PhaseValues<float>> temp_phase_volts_ = std::vector<PhaseValues<float>>(torque_steps_, {0.f, 0.f, 0.f});


        void target_selector(int direction = 1)
        {
            float gain = static_cast<float>(direction) * _2_PI_;
            float offset = 0.f;
            float shift = 0.f;

            if (direction == -1) {
                offset = _2_PI_;
                shift = 1.f;
            }

            for (size_t i = 0; i < COGGING_STEPS; ++i) {
                positions_.at(i) = offset + gain * (i + shift) / static_cast<float>(COGGING_STEPS);
                Serial.println(positions_.at(i));
            }

        }


        void cogging_mapper()
        {
        // Serial.println("Mappers!");
        float pos_target = positions_.at(idx);
        controller_.update_sensors();

        float error = normalize_angle(pos_target - controller_.get_shaft_radians());
        intl += (error * 1e-4);
        float torque = kP * error - kD * controller_.get_shaft_velocity() + kI * intl;
        torque = std::clamp(torque, -U2535.kT * 3.f, U2535.kT * 3.f);

        controller_.set_target(torque);
        controller_.update_control();

        if (fabs(error) < pos_err_tol && !locked) {
            locked = true;
        }
        if (locked) {
            if (fabs(error) > pos_err_tol) {
            locked = false;
            clk_start = 0;
            }
            temp_torques_.at(clk_start) = torque;
            temp_phase_volts_.at(clk_start) = controller_.get_last_phasevolts();
            clk_start++;
            if (clk_start >= torque_steps_) {
            float sum_ = 0.f;
            PhaseValues<float> volt_sum{0.f, 0.f, 0.f};
            for (size_t k = 0; k < torque_steps_; ++k) {
                sum_ += temp_torques_.at(k);
                volt_sum += temp_phase_volts_.at(k);
            }
            torques_.at(idx) = sum_ / static_cast<float>(torque_steps_);
            phase_volts_.at(idx) = volt_sum * (1.f / static_cast<float>(torque_steps_));
            locked = false;
            clk_start = 0;
            intl = 0.f;
            idx++;
            Serial.print("Target #");
            Serial.println(idx);
            }

            if (idx >= COGGING_STEPS) {
            report_out();
            }
        }
        }


        void report_out()
        {
        timer_.stop();
        Serial.println("=====");

        for (size_t j = 0; j < COGGING_STEPS; ++j) {
            Serial.print(positions_.at(j), 6);
            Serial.print("\t");
            Serial.print(torques_.at(j), 6);
            Serial.print("\t");
            Serial.print(phase_volts_.at(j).a, 6);
            Serial.print("\t");
            Serial.print(phase_volts_.at(j).b, 6);
            Serial.print("\t");
            Serial.println(phase_volts_.at(j).c, 6);
            Serial.flush();
        }
        delay(1);
        Serial.println("=====");
        Serial.flush();
        delay(10);
        Serial.print("Loop #");
        Serial.print(looper);
        Serial.println(" finished!");

        looper++;

        if (looper < max_loop) {
            idx = 0;
            controller_.start_control(100, false);
            timer_.begin([this] {cogging_mapper();}, 100);
            return;
        } 

        if (dir == 1)
        {
            dir = -1;
            looper = 0;
            target_selector(dir);
            idx = 0;
            controller_.start_control(100, false);
            timer_.begin([this] {cogging_mapper();}, 100);
            return;
        }

        Serial.println("Finished! Exiting");
        Serial.flush();
        delay(10);
        exit(1);

        }




};