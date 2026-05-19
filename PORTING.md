# NUControl Portability Survey

## Context

NUControl is a C++17 BLDC motor control library currently targeting Teensy 4.0 via PlatformIO + Arduino framework. The goal is to port it to the NXP RT1186 and, as part of that effort, extract the hardware-agnostic control math into a standalone C++ library that can be compiled for any target and unit-tested on a PC with Catch2.

This document is a **preliminary survey only** — categorizing existing files before any restructuring begins.

---

## Category 1: Hardware-Agnostic — Already Well-Abstracted

These files contain pure math, algorithms, and data structures. They depend only on `<math.h>` or the C++ standard library. They can be compiled and unit-tested on x86_64 today with zero modification.

| File | Contents |
|------|----------|
| [src/helpers.hpp](src/helpers.hpp) | Trig utilities, `normalize_angle()`, `_2_PI_` constants, `AngleTracker` |
| [src/transformations.hpp](src/transformations.hpp) | Clarke/Park transforms; `PhaseValues<T>`, `QuadDirectValues<T>` structs and operator overloads |
| [src/discrete_filter.hpp](src/discrete_filter.hpp) | Generic IIR `DiscreteFilter<T,C>`, `Butterworth2nd<T>`, `PIController<T>` |
| [src/motors.hpp](src/motors.hpp) | `MotorParameters` struct; `EC45_Flat`, `U2535` constants |
| [src/vel_filters.hpp](src/vel_filters.hpp) | Pre-computed Butterworth coefficients for 200 Hz velocity filtering |
| [src/anticog_helpers.hpp](src/anticog_helpers.hpp) | Anticogging lookup-table interpolation math |

**Verdict:** These six files are the core of the portable library. Move them as-is.

---

## Category 2: Should Be Hardware-Agnostic, But Isn't

These files contain valuable control logic that is tangled with Arduino/Teensy-specific APIs. The math is correct and reusable; the coupling is superficial and can be broken with targeted refactoring.

### `src/brushless_controller.hpp`
- **Good:** All FOC math (PID, coordinate transforms, back-EMF decoupling, feedforward), calibration logic, sensor fusion.
- **Bad:**
  - `#include <TeensyTimerTool.h>` — timer objects `ctrl_timer_`, `print_timer_` (`TeensyTimerTool::PeriodicTimer`) used only in `start_control()` / `stop_control()` / `start_print()`.
  - `delay()` calls inside `align_sensors()` calibration loop.
  - `Serial.print*` throughout calibration and debug methods (`align_sensors`, `print_calibration`, `load_calibration`).
  - Direct dependency on `driver.hpp`, `current_sense.hpp`, `encoder.hpp` (all Category 3).
- **Fix:** `control_step()` is already callable without the internal timer (via `use_internal_timer=false`). The timer glue, `delay()`, and `Serial` calls can be lifted to the platform layer.

### `src/cogging_mapper.hpp`
- **Good:** Position-PID cogging characterization algorithm; accumulates torque/position tables.
- **Bad:** `TeensyTimerTool::PeriodicTimer timer_` drives the mapping loop; same timer coupling as above.
- **Fix:** Same as controller — expose `cogging_mapper()` as a callable step; move timer ownership to the platform.

### `src/telemetry.hpp`
- **Good:** JSON serialization of motor state snapshots; self-contained `MotorTelemetry` struct with sensible fields.
- **Bad:** `#include <Arduino.h>` for `micros()` and `Stream`.
- **Fix:** Replace `micros()` with an injected `uint32_t (*clock_us)()` function pointer (or `std::function`). Replace `Stream&` with `std::ostream&` or a write-callback — then serialize works on any target.

### `src/userConfig.h`
- Currently empty placeholder, but `#include "boardDef.h"` pulls in TeensyTimerTool board definitions.
- **Fix:** Delete the include; make it truly empty or remove the file.

---

## Category 3: Hardware-Specific — Move to `platforms/` or `demos/`

These files are permanently tied to Teensy/Arduino hardware. They should leave the core library.

| File | Why Hardware-Specific | Destination |
|------|-----------------------|-------------|
| [src/driver.hpp](src/driver.hpp) | `analogWrite()`, `analogWriteFrequency()`, `digitalWriteFast()`, `pinMode()` — direct Teensy PWM/GPIO | `platforms/teensy40/` |
| [src/encoder.hpp](src/encoder.hpp) | `#include <Arduino.h>`, `#include <imxrt.h>` — abstract base class but with MCU-specific includes | `platforms/teensy40/` |
| [src/spi_encoder.hpp](src/spi_encoder.hpp) | `SPIClass`, `SPISettings`, `delayNanoseconds()`, `digitalWriteFast()` | `platforms/teensy40/` |
| [src/current_sense.hpp](src/current_sense.hpp) | `analogRead()`, `analogReadRes()`, `pinMode()`, `Serial.*`, `delay()` | `platforms/teensy40/` |
| [src/errors.hpp](src/errors.hpp) | `#include <Arduino.h>`, `TeensyTimerTool::errorCode`, `Serial.*`, `exit()` | `platforms/teensy40/` |
| [src/demos/one_shot.cpp](src/demos/one_shot.cpp) | Full Teensy hardware instantiation, hardcoded pins | `platforms/teensy40/demos/` |
| [src/demos/position_sweep.cpp](src/demos/position_sweep.cpp) | Full Teensy hardware instantiation, hardcoded pins | `platforms/teensy40/demos/` |

**Note:** `encoder.hpp` defines the abstract `AbsoluteEncoder` base class that `BrushlessController` depends on. For true portability, this interface must be extracted into the core library (Category 1) and the Teensy-specific includes removed.

---

## Incomplete / Commented-Out Files

| File | Status |
|------|--------|
| `src/brushed_controller.hpp` | Fully commented out; not yet implemented |
| `src/chirp_controller.hpp` | Fully commented out; experimental |

These can be deferred — they don't affect portability planning.

---

## Summary

| Category | Files | Action |
|----------|-------|--------|
| 1 — Portable now | `helpers`, `transformations`, `discrete_filter`, `motors`, `vel_filters`, `anticog_helpers` | Move to portable library as-is |
| 2 — Portable after refactor | `brushless_controller`, `cogging_mapper`, `telemetry`, `userConfig` | Decouple timer/Serial/delay; inject clock + I/O |
| 3 — Hardware-specific | `driver`, `encoder`, `spi_encoder`, `current_sense`, `errors`, 2 demos | Move to `platforms/teensy40/` |

The biggest single refactor is `brushless_controller.hpp`: separating `control_step()` (pure math, already callable standalone) from timer ownership and debug I/O unlocks almost everything else.
