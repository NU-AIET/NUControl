/**
 * @file cogging_mapper.hpp
 * @brief Automated cogging-torque measurement sweep for `AnticoggingCompensator`.
 *
 * @details `CoggingMapper` drives a `BrushlessController` through a series of
 * evenly-spaced angular set-points and records the torque and phase voltages
 * required to hold each position in steady state. The resulting arrays are the
 * input to `AnticoggingCompensator` (see `anticog_helpers.hpp`).
 *
 * ## Mapping procedure (`map_cogging()`)
 *
 * 1. **Direction pass 1** (CW, `dir = +1`): visits `steps_` target positions
 *    uniformly distributed over [0, 2π) in increasing order.
 * 2. **Direction pass 2** (CCW, `dir = −1`): revisits the same `steps_`
 *    positions in decreasing order to capture hysteresis.
 * 3. Each direction pass is repeated `loops` times and the results are averaged.
 *
 * ### Per-target state machine
 * At each target position, a PD+I controller drives the shaft to within
 * `pos_err_tol` = 0.01 rad, then accumulates `torque_steps_` = 1000 samples
 * (100 ms at 10 kHz) of steady-state torque and phase voltage. If the motor
 * fails to reach the target within `timeout_max_` control ticks, the target
 * is marked with `timeout_val_` = −127.0.
 *
 * ### PD+I controller
 * @f[
 *   \tau = K_p\,e + K_i\,\int e\,dt - K_d\,\dot\theta
 * @f]
 * Default gains (U2535): @f$ K_p = 1.0,\ K_i = 0.5,\ K_d = 0.1 @f$.
 * The integral term is clamped to ±`max_intl_` to prevent windup.
 *
 * ### Output
 * After all passes complete, `report_out()` prints a TSV table to `Serial`:
 * ```
 * =====
 * <angle_rad>  <torque_Nm>  <Va_V>  <Vb_V>  <Vc_V>
 * …
 * =====
 * ```
 * Copy this output into `anticog_torque_map` / `anticog_volt_map` arrays and
 * pass them to `AnticoggingCompensator`.
 *
 * @note The mapper runs in its own `TeensyTimerTool::TCK` timer at 100 µs
 *       (10 kHz) — the same rate as the main control loop. It calls
 *       `controller_.update_sensors()` and `controller_.update_control()`
 *       directly. Do not run the main control ISR simultaneously.
 *
 * @warning `CoggingMapper` calls `exit(1)` when all passes are complete.
 *          This is intentional — the mapping run is destructive to ongoing
 *          control and is meant to be used in a dedicated calibration firmware
 *          build, not in production.
 */

#include "brushless_controller.hpp"
#include <array>


/**
 * @brief Automated cogging-torque and phase-voltage mapping sweep.
 *
 * @details Drives a `BrushlessController` through `steps_` angular set-points,
 * records steady-state torque and phase voltages at each point, and reports
 * results over Serial for import into `AnticoggingCompensator`.
 *
 * @tparam steps_ Number of measurement points per direction per loop.
 *                Uniformly distributed over one full mechanical revolution.
 *                Typical: 200–1000.
 */
template<std::size_t steps_>
class CoggingMapper
{
    public:
        CoggingMapper() = default;
        ~CoggingMapper() = default;

        /**
         * @brief Constructs the mapper bound to a specific controller.
         * @param controller Reference to the `BrushlessController` to sweep.
         *                   Must outlive this `CoggingMapper` instance.
         */
        CoggingMapper(BrushlessController & controller)
        : controller_(controller),
          timer_(TeensyTimerTool::TCK)
        {};

        /**
         * @brief Starts the cogging-map sweep.
         *
         * @details Initialises the sweep state, optionally disables existing
         * anticogging compensation (so only raw cogging is captured), starts
         * the controller, and launches the `TeensyTimerTool` timer at 100 µs.
         *
         * The sweep runs autonomously via the timer callback. Blocking I/O
         * (Serial output) only occurs at the end of each loop in `report_out()`.
         *
         * @param loops          Number of sweeps per direction (CW and CCW).
         *                       Results from all loops are reported; the caller
         *                       should average them offline.
         * @param disable_cogging If `true` (default), calls `controller_.disable_anticog()`
         *                        before starting so that the map captures raw cogging
         *                        without feedforward interference.
         *
         * @warning Calling `map_cogging()` starts an autonomous control loop.
         *          Any concurrent use of the same `BrushlessController` will
         *          corrupt the sweep.
         */
        void map_cogging(int loops, bool disable_cogging = true)
        {
            max_loop = loops;
            looper = 0;
            idx = 0;
            target_selector(dir);
            if(disable_cogging){ controller_.disable_anticog();}
            auto motor = controller_.get_motor();
            max_torque = 0.5f * motor.kT * motor.SAFE_CURRENT;
            controller_.start_control(100, false);
            timer_.begin([this] {cogging_mapper();}, 100);
        }

        /**
         * @brief Configures the per-target timeout behaviour.
         *
         * @details If `state` is `true`, a target that is not reached within
         * `timeout_max` control ticks is marked with `timeout_val` and the sweep
         * moves on. This prevents a stalled motor from blocking the entire map.
         *
         * @param state       `true` to enable timeout (default), `false` to wait forever.
         * @param timeout_max Number of 100 µs ticks before timeout (default: 200000 = 20 s).
         * @param timeout_val Sentinel value written to `torques_` and `phase_volts_`
         *                    on timeout (default: −127.0). Easily identified in post-processing.
         */
        void set_timeout_state(bool state, size_t timeout_max=200000, float timeout_val=-127.f){
            timeout_state_ = state;
            timeout_max_ = timeout_max;
            timeout_val_ = timeout_val;
        }

        /**
         * @brief Overrides the default PD+I position-controller gains.
         *
         * @details Default gains are tuned for the T-Motor U2535. Adjust for
         * different motors by starting with `kp` ≈ 0.5–2× the plant gain and
         * `kd` ≈ 0.05–0.2× `kp`. High `ki` can cause overshoot and missed targets.
         *
         * @param kp       Proportional gain [Nm/rad].
         * @param ki       Integral gain [Nm/(rad·s)].
         * @param kd       Derivative gain [Nm·s/rad] (acts on velocity).
         * @param intl_max Integral clamp magnitude [Nm] (anti-windup).
         */
        void set_controller_gains(float kp, float ki, float kd, float intl_max)
        {
            kP_ = kp;
            kI_ = ki;
            kD_ = kd;
            max_intl_ = intl_max;
        }

    private:

        BrushlessController & controller_; ///< Bound controller instance.
        TeensyTimerTool::PeriodicTimer timer_; ///< TCK timer driving the sweep callback.

        std::array<float, steps_> positions_{}; ///< Target angle for each step [rad].
        std::array<float, steps_> torques_{};   ///< Measured steady-state torque [Nm].
        std::array<PhaseValues<float>,steps_> phase_volts_{}; ///< Measured steady-state phase voltages [V].

        float pos_target = 0.f;
        size_t idx = 0;

        bool   timeout_state_ = true;    ///< Timeout enabled flag.
        float  timeout_val_   = -127.f;  ///< Sentinel for timed-out targets.
        size_t timeout_clk    = 0;       ///< Ticks elapsed at current target.
        size_t timeout_max_   = 200000;  ///< Tick limit before timeout (default: 20 s).

        // PD+I controller state
        float kP_ = 1.f;    ///< Proportional gain [Nm/rad].
        float kD_ = 0.1f;   ///< Derivative gain [Nm·s/rad].
        float kI_ = 0.5f;   ///< Integral gain [Nm/(rad·s)].

        float intl      = 0.f;   ///< Integral accumulator [Nm].
        float max_intl_ = 0.25f; ///< Integral clamp [Nm].
        float max_torque = 0.5f; ///< Peak torque limit for sweep [Nm] = 0.5 × kT × SAFE_CURRENT.

        int dir      = 1;  ///< Current sweep direction: +1 (CW) or −1 (CCW).
        int max_loop = 10; ///< Loops per direction.
        int looper   = 0;  ///< Current loop index.

        const float dt_ = 100.f * 1e-6f; ///< Control period [s] = 100 µs.

        size_t clk_start    = 0;    ///< Ticks spent in steady-state accumulation.
        float  pos_err_tol  = 0.01f; ///< Position error threshold to enter steady-state [rad].
        bool   locked       = false; ///< True when shaft is within `pos_err_tol`.

        /// Number of control ticks to average once locked (100 ms at 10 kHz).
        constexpr static size_t torque_steps_ = 1000;
        constexpr static float  step_inv = 1.f / static_cast<float>(torque_steps_);

        float              torque_sum_ = 0.f;              ///< Running torque average.
        PhaseValues<float> volt_sum_{0.f, 0.f, 0.f};       ///< Running phase-voltage average.

        /**
         * @brief Populates `positions_` with uniformly-spaced targets for one direction.
         *
         * @details For CW (`direction = +1`): targets = 2π × i/steps_ for i = 0…steps_−1.
         * For CCW (`direction = −1`): targets = 2π × (steps_−1−i)/steps_ (decreasing order).
         *
         * @param direction +1 for CW, −1 for CCW.
         */
        void target_selector(int direction = 1)
        {
            float gain = static_cast<float>(direction) * _2_PI_;
            float offset = 0.f;
            float shift = 0.f;

            if (direction == -1) {
                offset = _2_PI_;
                shift = 1.f;
            }

            for (size_t i = 0; i < steps_; ++i) {
                positions_.at(i) = offset + gain * (i + shift) / static_cast<float>(steps_);
                Serial.println(positions_.at(i));
            }
        }


        /**
         * @brief Timer callback — runs one control tick of the sweep state machine.
         *
         * @details Called at 10 kHz by the `TeensyTimerTool` TCK timer. Implements
         * the per-target PD+I controller and the lock/accumulate/advance state machine.
         * When `idx` reaches `steps_`, calls `report_out()` to print results and
         * advance to the next loop or direction.
         */
        void cogging_mapper()
        {
            controller_.update_sensors();
            float error = normalize_angle(positions_.at(idx) - controller_.get_shaft_angle());
            intl += (error * dt_);
            intl = std::clamp(intl, -max_intl_, max_intl_);
            float torque = kP_ * error - kD_ * controller_.get_shaft_velocity() + kI_ * intl;
            torque = std::clamp(torque, -max_torque, max_torque);
            controller_.set_target(torque);
            controller_.update_control();
            timeout_clk++;

            if(timeout_state_ && timeout_clk >= timeout_max_){
                timeout_clk = 0;
                torques_.at(idx) = timeout_val_;
                phase_volts_.at(idx) = {timeout_val_, timeout_val_, timeout_val_};
                volt_sum_ = 0.f;
                torque_sum_ = 0.f;
                locked = false;
                clk_start = 0;
                intl = 0.f;
                Serial.println("Target Timedout");
                idx++;
                Serial.print("Target #");
                Serial.println(idx);
            }
        if (fabs(error) < pos_err_tol && !locked) { locked = true; }
        if (locked) {
            if (fabs(error) > pos_err_tol) {
                locked = false;
                volt_sum_ = 0.f;
                torque_sum_ = 0.f;
                clk_start = 0;
            }
            torque_sum_ += torque * step_inv;
            volt_sum_ += controller_.get_last_phasevolts() * step_inv;
            clk_start++;
            if (clk_start >= torque_steps_) {
                timeout_clk = 0;
                torques_.at(idx) = torque_sum_;
                phase_volts_.at(idx) = volt_sum_;
                volt_sum_ = 0.f;
                torque_sum_ = 0.f;
                locked = false;
                clk_start = 0;
                intl = 0.f;
                idx++;
                Serial.print("Target #");
                Serial.println(idx);
            }
        }

            if (idx >= steps_) {
                report_out();
            }
        }

        /**
         * @brief Prints the completed loop's results over Serial and advances the sweep.
         *
         * @details Output format (TSV, one row per step):
         * ```
         * =====
         * <angle_rad>  <torque_Nm>  <Va_V>  <Vb_V>  <Vc_V>
         * …
         * =====
         * ```
         * After printing:
         * - If more CW loops remain, restarts `cogging_mapper()` immediately.
         * - If all CW loops are done, switches to CCW (`dir = −1`) and restarts.
         * - If both directions are complete, calls `exit(1)`.
         *
         * @warning Calls `exit(1)` on completion. Intended for standalone
         *          calibration firmware only — not suitable for use in production builds.
         */
        void report_out()
        {
        controller_.stop_control();
        timer_.stop();
        Serial.println("=====");

        for (size_t j = 0; j < steps_; ++j) {
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
            delay(1);
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
