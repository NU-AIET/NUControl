/**
 * @file nu_log.hpp
 * @brief ISR-safe, non-blocking structured logger for NUControl.
 *
 * @details All code paths — including the 10 kHz control ISR — call
 * `nu_log::push()` to enqueue a timestamped `Entry` into a lock-free
 * single-producer / single-consumer ring buffer.  The application's main
 * loop calls `nu_log::drain()` to dequeue and print entries over Serial
 * (or any `Print`-compatible sink).
 *
 * ## Quick start
 * ```cpp
 * // From anywhere (ISR-safe):
 * nu_log::push(nu_log::Level::WARN, nu_log::Id::CURRENT_SATURATED, amps);
 *
 * // From the main loop:
 * void loop() {
 *     nu_log::drain(Serial);
 * }
 * ```
 *
 * ## Design constraints
 * - No heap allocation after construction.
 * - `push()` is wait-free and safe to call from any ISR.
 * - SPSC: exactly one writer (may be the control ISR), one reader (main loop).
 * - Entries are silently dropped when the buffer is full; the `dropped`
 *   counter records the count so the next `drain()` can report it.
 * - `micros()` is safe to call from ISR on Teensy 4.x (reads DWT->CYCCNT).
 *
 * @note `nu_log.hpp` is included by `nu_control.hpp` and transitively by
 *       every library header that uses logging.  User code needs only
 *       `#include "nu_control.hpp"` and a `nu_log::drain(Serial)` call in
 *       `loop()`.
 */

#ifndef NU_LOG_HPP
#define NU_LOG_HPP

#include <Arduino.h>
#include <atomic>
#include <cstdint>
#include <cstdio>

namespace nu_log {

// ── Severity levels ───────────────────────────────────────────────────────────

enum class Level : uint8_t { DEBUG = 0, INFO, WARN, ERROR };

// ── Message identifiers ───────────────────────────────────────────────────────
// Each Id maps to a human-readable string in the drain() lookup table.
// The associated value field conveys the most diagnostic scalar for that event.

enum class Id : uint8_t {
  // --- ISR-safe: current sensing (may fire every 100 µs) -------------------
  CURRENT_SATURATED   = 0,  ///< WARN  | value = |I| [A]
  CURRENT_OVER_LIMIT  = 1,  ///< ERROR | value = |I| [A]
  CS_NOT_ALIGNED      = 2,  ///< WARN  | value = 0 (alignment skipped)

  // --- Sensor initialisation -----------------------------------------------
  CS_SENSOR_COUNT     = 3,  ///< INFO  | value = sensor count
  CS_OFFSET_FAIL      = 4,  ///< ERROR | value = measured offset [V]
  CS_OFFSET_DEVIATION = 5,  ///< WARN  | value = measured offset [V]

  // --- Sensor-to-phase alignment -------------------------------------------
  CS_ALIGN_SENSOR_IDX = 6,  ///< INFO  | value = sensor index
  CS_ALIGN_READ_A     = 7,  ///< INFO  | value = phase-A current [A]
  CS_ALIGN_READ_B     = 8,  ///< INFO  | value = phase-B current [A]
  CS_ALIGN_READ_C     = 9,  ///< INFO  | value = phase-C current [A]
  CS_NO_CURRENT       = 10, ///< ERROR | value = sensor index
  CS_DMA_PIN_INVALID  = 11, ///< WARN  | value = Arduino pin number

  // --- Calibrated phase mapping --------------------------------------------
  CAL_CS_PHASE_IDX_A  = 12, ///< INFO  | value = sensor index for phase A
  CAL_CS_PHASE_IDX_B  = 13, ///< INFO  | value = sensor index for phase B
  CAL_CS_PHASE_IDX_C  = 14, ///< INFO  | value = sensor index for phase C
  CAL_CS_PHASE_DIR_A  = 15, ///< INFO  | value = direction for phase A (+1/-1)
  CAL_CS_PHASE_DIR_B  = 16, ///< INFO  | value = direction for phase B (+1/-1)
  CAL_CS_PHASE_DIR_C  = 17, ///< INFO  | value = direction for phase C (+1/-1)

  // --- Controller initialisation -------------------------------------------
  PWM_CONFIG_FAIL     = 18, ///< WARN  | value = 0
  ADC_TRIGGER_FAIL    = 19, ///< WARN  | value = 0

  // --- Controller calibration ----------------------------------------------
  INVALID_DIRECTION   = 20, ///< WARN  | value = supplied direction value
  ALIGN_FAIL          = 21, ///< ERROR | value = 0
  ALIGN_SCAN_START    = 22, ///< INFO  | value = +1 (forward) or -1 (reverse)
  NO_MOTION_DETECTED  = 23, ///< ERROR | value = init_angle [rad]
  ALIGN_FINAL_ANGLE   = 24, ///< INFO  | value = final_angle [rad]
  ENCODER_DIRECTION   = 25, ///< INFO  | value = direction (+1/-1)
  ZERO_ELEC_ANGLE     = 26, ///< INFO  | value = e_ang_offset [rad]
  RL_RESULT_R         = 27, ///< INFO  | value = R [Ω]
  RL_RESULT_L         = 28, ///< INFO  | value = L [µH]
  RL_RESULT_DI        = 29, ///< INFO  | value = ΔI_avg [A]
  RL_RESULT_OK        = 30, ///< INFO  | value = 1.0 (success) / 0.0 (fail)

  // --- Fatal fault callbacks (errors.hpp) ----------------------------------
  FAULT_CURRENT_LIMIT = 31, ///< ERROR | value = 0
  FAULT_CS_INIT       = 32, ///< ERROR | value = 0
  FAULT_DRIVER_INIT   = 33, ///< ERROR | value = 0
  FAULT_CS_ALIGN      = 34, ///< ERROR | value = 0
  FAULT_TIMER         = 35, ///< ERROR | value = TeensyTimerTool error code

  // --- Periodic telemetry --------------------------------------------------
  VELOCITY_SNAPSHOT   = 36, ///< INFO  | value = shaft_velocity [rad/s]

  _COUNT = 37
};


// ── Entry struct ──────────────────────────────────────────────────────────────

struct Entry {
  Level    level;    ///< Severity
  Id       id;       ///< Message identifier (maps to a human-readable string)
  float    value;    ///< Diagnostic scalar for this event
  uint32_t tick_us;  ///< Timestamp from micros() at time of push
};


// ── Lock-free SPSC ring buffer ────────────────────────────────────────────────

/// Buffer capacity (must be a power of 2).
static constexpr size_t CAPACITY = 256;
static_assert((CAPACITY & (CAPACITY - 1)) == 0, "CAPACITY must be a power of 2");

struct RingBuffer {
  Entry               buf[CAPACITY];
  std::atomic<size_t> head{0};  ///< Written by producer (ISR)
  std::atomic<size_t> tail{0};  ///< Written by consumer (main loop)
  std::atomic<uint32_t> dropped{0}; ///< Entries lost due to full buffer

  /// @brief Enqueue one entry.  Wait-free; safe from any ISR.
  /// @returns true if the entry was accepted, false if the buffer was full.
  bool push(Entry e) {
    const size_t h    = head.load(std::memory_order_relaxed);
    const size_t next = (h + 1) & (CAPACITY - 1);
    if (next == tail.load(std::memory_order_acquire)) {
      dropped.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    buf[h] = e;
    head.store(next, std::memory_order_release);
    return true;
  }

  /// @brief Dequeue one entry into `out`.
  /// @returns false when the buffer is empty.
  bool pop(Entry& out) {
    const size_t t = tail.load(std::memory_order_relaxed);
    if (t == head.load(std::memory_order_acquire)) return false;
    out = buf[t];
    tail.store((t + 1) & (CAPACITY - 1), std::memory_order_release);
    return true;
  }
};

/// Global log buffer.  One instance shared by all modules.
inline RingBuffer g_log;


// ── Push helper ───────────────────────────────────────────────────────────────

/**
 * @brief Enqueues a log entry.  ISR-safe, wait-free.
 *
 * @param level  Severity level.
 * @param id     Message identifier (see `nu_log::Id`).
 * @param value  Diagnostic scalar value. Default 0.
 */
inline bool push(Level level, Id id, float value = 0.f) {
  return g_log.push({level, id, value, static_cast<uint32_t>(micros())});
}


// ── Drain ────────────────────────────────────────────────────────────────────

/**
 * @brief Dequeues all pending log entries and prints them.
 *
 * @details Call from the main loop (or any non-ISR context).  Each entry is
 * formatted as:
 * @code
 * [WARN]  t= 12345us  Current sensor saturated: 3.2500 A
 * @endcode
 *
 * @param out  Output sink.  Defaults to `Serial`.
 */
inline void drain(Print& out = Serial) {
  // Message strings indexed by nu_log::Id.
  static const char* const MSGS[] = {
    /* 0  CURRENT_SATURATED   */ "Current sensor saturated",
    /* 1  CURRENT_OVER_LIMIT  */ "Current sensor over limit",
    /* 2  CS_NOT_ALIGNED      */ "Sensor package not aligned to driver",
    /* 3  CS_SENSOR_COUNT     */ "Current sensor count",
    /* 4  CS_OFFSET_FAIL      */ "Sensor offset validation FAILED — ideal 1.65 V, got",
    /* 5  CS_OFFSET_DEVIATION */ "Sensor offset deviates from ideal (1.65 V) —",
    /* 6  CS_ALIGN_SENSOR_IDX */ "CS align — sensor index",
    /* 7  CS_ALIGN_READ_A     */ "CS align — phase-A read [A]",
    /* 8  CS_ALIGN_READ_B     */ "CS align — phase-B read [A]",
    /* 9  CS_ALIGN_READ_C     */ "CS align — phase-C read [A]",
    /* 10 CS_NO_CURRENT       */ "CS align — no current on sensor index",
    /* 11 CS_DMA_PIN_INVALID  */ "DMA init — pin does not map to ADC channel",
    /* 12 CAL_CS_PHASE_IDX_A  */ "CS calibration — phase-A sensor index",
    /* 13 CAL_CS_PHASE_IDX_B  */ "CS calibration — phase-B sensor index",
    /* 14 CAL_CS_PHASE_IDX_C  */ "CS calibration — phase-C sensor index",
    /* 15 CAL_CS_PHASE_DIR_A  */ "CS calibration — phase-A direction",
    /* 16 CAL_CS_PHASE_DIR_B  */ "CS calibration — phase-B direction",
    /* 17 CAL_CS_PHASE_DIR_C  */ "CS calibration — phase-C direction",
    /* 18 PWM_CONFIG_FAIL     */ "Center-aligned PWM config failed — falling back to edge-aligned + analogRead()",
    /* 19 ADC_TRIGGER_FAIL    */ "ADC DMA trigger config failed — falling back to analogRead()",
    /* 20 INVALID_DIRECTION   */ "Invalid direction argument (must be +1 or -1)",
    /* 21 ALIGN_FAIL          */ "Sensor-to-phase alignment failed",
    /* 22 ALIGN_SCAN_START    */ "Alignment scan direction (+1=fwd, -1=rev)",
    /* 23 NO_MOTION_DETECTED  */ "No encoder motion detected — init angle [rad]",
    /* 24 ALIGN_FINAL_ANGLE   */ "Alignment scan — final angle [rad]",
    /* 25 ENCODER_DIRECTION   */ "Encoder direction",
    /* 26 ZERO_ELEC_ANGLE     */ "Zero electrical angle [rad]",
    /* 27 RL_RESULT_R         */ "identify_rl — R [Ohm]",
    /* 28 RL_RESULT_L         */ "identify_rl — L [uH]",
    /* 29 RL_RESULT_DI        */ "identify_rl — dI_avg [A]",
    /* 30 RL_RESULT_OK        */ "identify_rl — success (1=yes, 0=no)",
    /* 31 FAULT_CURRENT_LIMIT */ "FAULT: current sensor exceeds safe limit",
    /* 32 FAULT_CS_INIT       */ "FAULT: current sensors failed to initialise",
    /* 33 FAULT_DRIVER_INIT   */ "FAULT: gate driver failed to initialise",
    /* 34 FAULT_CS_ALIGN      */ "FAULT: current sensors failed to align to phases",
    /* 35 FAULT_TIMER         */ "FAULT: timer error code",
    /* 36 VELOCITY_SNAPSHOT   */ "velocity [rad/s]",
  };
  static_assert(sizeof(MSGS) / sizeof(MSGS[0]) == static_cast<size_t>(Id::_COUNT),
                "MSGS table length must match Id::_COUNT");

  static const char* const LEVEL_TAGS[] = { "[DEBUG]", "[INFO] ", "[WARN] ", "[ERROR]" };

  // Report any dropped entries first.
  uint32_t n_dropped = g_log.dropped.exchange(0, std::memory_order_relaxed);
  if (n_dropped) {
    char buf[64];
    snprintf(buf, sizeof(buf), "[WARN]  *** %u log entries dropped (buffer full) ***\n",
             static_cast<unsigned>(n_dropped));
    out.print(buf);
  }

  Entry e;
  char  line[96];
  while (g_log.pop(e)) {
    const char* tag = (static_cast<uint8_t>(e.level) < 4)
                      ? LEVEL_TAGS[static_cast<uint8_t>(e.level)]
                      : "[?????]";
    const char* msg = (static_cast<size_t>(e.id) < static_cast<size_t>(Id::_COUNT))
                      ? MSGS[static_cast<size_t>(e.id)]
                      : "unknown message id";
    snprintf(line, sizeof(line), "%s t=%7luus  %s: %.4g\n",
             tag,
             static_cast<unsigned long>(e.tick_us),
             msg,
             static_cast<double>(e.value));
    out.print(line);
  }
}

} // namespace nu_log

#endif // NU_LOG_HPP
