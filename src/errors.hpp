/**
 * @file errors.hpp
 * @brief Error and warning codes with associated handler callbacks.
 *
 * @details Defines `ErrorCodes` and `WarningCodes` enumerations and three
 * handler functions:
 * - `print_errors()` — pushes a human-readable description to `nu_log`.
 * - `handle_errors()` — pushes, drains to Serial, flushes, then halts.
 * - `timer_errors()` — TeensyTimerTool error callback; pushes, drains, halts.
 *
 * @warning `handle_errors()` calls `exit(0)`, which is **not recoverable**
 * on bare-metal Teensy. The device must be power-cycled or reset after a fault.
 */

#ifndef ERRORS_HPP
#define ERRORS_HPP

#include "nu_log.hpp"


/**
 * @brief Non-fatal warning conditions that should be logged but do not halt execution.
 */
enum WarningCodes
{
  CURRENT_SENSE_SATURATION,          ///< ADC reading exceeds 1.5× gain; sensor near rail.
  DRIVER_VOLTAGE_EXCEEDS_USER_LIMIT, ///< Voltage command exceeded the configured MAX_VOLT_.
};

/**
 * @brief Fatal error conditions that trigger `handle_errors()` and halt the program.
 */
enum ErrorCodes
{
  CURRENT_SENSE_OVER_LIMIT,    ///< Phase current exceeds 2× gain; risk of hardware damage.
  DRIVER_INIT_FAIL,            ///< Gate driver failed to initialise (pin config error).
  CURRENT_SENSE_INIT_FAIL,     ///< Current sensor offset out of tolerance (±0.5 V from 1.65 V).
  CURRENT_SENSE_ALIGN_FAIL,    ///< No detectable current during sensor-to-phase alignment.
};

/**
 * @brief Pushes a human-readable error description to the log buffer.
 *
 * @details Non-blocking; safe to call from ISR.  Does NOT drain or halt.
 * The caller is responsible for calling `nu_log::drain()` when appropriate.
 *
 * @param code The error code to describe.
 */
void print_errors(ErrorCodes code)
{
  switch (code) {
    case ErrorCodes::CURRENT_SENSE_OVER_LIMIT:
      nu_log::push(nu_log::Level::ERROR, nu_log::Id::FAULT_CURRENT_LIMIT);
      break;
    case ErrorCodes::CURRENT_SENSE_INIT_FAIL:
      nu_log::push(nu_log::Level::ERROR, nu_log::Id::FAULT_CS_INIT);
      break;
    case ErrorCodes::DRIVER_INIT_FAIL:
      nu_log::push(nu_log::Level::ERROR, nu_log::Id::FAULT_DRIVER_INIT);
      break;
    case ErrorCodes::CURRENT_SENSE_ALIGN_FAIL:
      nu_log::push(nu_log::Level::ERROR, nu_log::Id::FAULT_CS_ALIGN);
      break;
  }
}

/**
 * @brief Logs the error, immediately drains to Serial, flushes, then halts.
 *
 * @details Calls `print_errors()` (which enqueues to `nu_log`), then
 * immediately drains the buffer to `Serial` and flushes to guarantee the
 * message appears before `exit(0)` is called.
 *
 * @param code The error code to handle.
 *
 * @warning Calling this function is **not recoverable**. The MCU must be
 * power-cycled or reset. `exit()` on Teensy jumps to the reset vector.
 */
void handle_errors(ErrorCodes code)
{
  print_errors(code);
  nu_log::drain(Serial);
  Serial.flush();
  delay(10);
  exit(0);
}

/**
 * @brief TeensyTimerTool error callback — logs the timer error code and halts.
 *
 * @details Registered via `TeensyTimerTool::attachErrFunc(timer_errors)` in
 * `setup()`. Enqueues a `FAULT_TIMER` entry with the numeric error code,
 * drains and flushes before halting.
 *
 * @param err TeensyTimerTool internal error code.
 *
 * @warning Halts execution after a 1 s delay via `exit(0)`.
 */
void timer_errors(TeensyTimerTool::errorCode err)
{
  nu_log::push(nu_log::Level::ERROR, nu_log::Id::FAULT_TIMER,
               static_cast<float>(err));
  nu_log::drain(Serial);
  Serial.flush();
  delay(1000);
  exit(0);
}


#endif // ERRORS_HPP
