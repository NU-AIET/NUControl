# NUControl Portability Survey

## Context

NUControl is a C++17 BLDC motor control library being ported from Teensy 4.0 to the NXP RT1186. The goal is to extract all hardware-agnostic control math into `core/` — a standalone header-only C++ library compilable for any target and unit-testable on a PC with Catch2.

---

## Current State

### `core/` — Portable library (done)

These files are in the CMake `nucontrol_core` INTERFACE target. They depend only on `<math.h>` and the C++ standard library. Unit tests exist for all of them.

| File | Contents | Tests |
|------|----------|-------|
| [core/helpers.hpp](core/helpers.hpp) | Trig utilities, `normalize_angle()`, `_2_PI_` constants | `tests/test_helpers.cpp` |
| [core/transformations.hpp](core/transformations.hpp) | Clarke/Park transforms; `PhaseValues<T>`, `QuadDirectValues<T>` | `tests/test_transformations.cpp` |
| [core/discrete_filter.hpp](core/discrete_filter.hpp) | Generic IIR `DiscreteFilter<T,C>`, `Butterworth2nd<T>`, `PIController<T>` | — |
| [core/motors.hpp](core/motors.hpp) | `MotorParameters` struct; `EC45_Flat`, `U2535` constants | — |
| [core/vel_filters.hpp](core/vel_filters.hpp) | Pre-computed Butterworth coefficients for 200 Hz velocity filtering | — |
| [core/anticog_helpers.hpp](core/anticog_helpers.hpp) | Anticogging lookup-table interpolation math | `tests/test_anticog.cpp` |

### `platforms/teensy40/` — Teensy PlatformIO project

Currently holds two distinct categories of code that need to be separated:

#### Category 2: Should be portable — needs refactoring

These files contain valuable control logic tangled with Arduino/Teensy-specific APIs. The math is correct and reusable; the coupling is superficial.

| File | What's good | What's blocking portability |
|------|-------------|----------------------------|
| [platforms/teensy40/brushless_controller.hpp](platforms/teensy40/brushless_controller.hpp) | All FOC math (PID, transforms, back-EMF, feedforward), calibration logic | `#include <TeensyTimerTool.h>`; `delay()` in calibration; `Serial.print*` in debug methods; depends on Category 3 types |
| [platforms/teensy40/cogging_mapper.hpp](platforms/teensy40/cogging_mapper.hpp) | Position-PID cogging characterization algorithm | `TeensyTimerTool::PeriodicTimer` drives the mapping loop |
| [platforms/teensy40/telemetry.hpp](platforms/teensy40/telemetry.hpp) | JSON serialization of motor state snapshots | `#include <Arduino.h>` for `micros()` and `Stream` |
| [platforms/teensy40/userConfig.h](platforms/teensy40/userConfig.h) | User config placeholder | `#include "boardDef.h"` pulls in Teensy timer definitions |

**Fix strategy:**
- `brushless_controller.hpp`: `control_step()` is already callable without the internal timer (`use_internal_timer=false`). Extract abstract interfaces (`AbsoluteEncoder`, `BrushlessDriver`, `CurrentSensor`) into `core/`; move timer glue, `delay()`, and `Serial` calls to the platform layer.
- `cogging_mapper.hpp`: Same pattern — expose `cogging_mapper()` as a callable step, move timer ownership to platform.
- `telemetry.hpp`: Replace `micros()` with an injected `uint32_t (*clock_us)()` function pointer; replace `Stream&` with `std::ostream&` or a write-callback.
- `userConfig.h`: Remove the `boardDef.h` include or delete the file entirely.

#### Category 3: Permanently hardware-specific — stays in `platforms/teensy40/`

| File | Why hardware-specific |
|------|-----------------------|
| [platforms/teensy40/driver.hpp](platforms/teensy40/driver.hpp) | `analogWrite()`, `analogWriteFrequency()`, `digitalWriteFast()`, `pinMode()` |
| [platforms/teensy40/encoder.hpp](platforms/teensy40/encoder.hpp) | `#include <Arduino.h>`, `#include <imxrt.h>`; also defines `AbsoluteEncoder` interface (needs extraction — see below) |
| [platforms/teensy40/spi_encoder.hpp](platforms/teensy40/spi_encoder.hpp) | `SPIClass`, `SPISettings`, `delayNanoseconds()`, `digitalWriteFast()` |
| [platforms/teensy40/current_sense.hpp](platforms/teensy40/current_sense.hpp) | `analogRead()`, `analogReadRes()`, `pinMode()`, `Serial.*`, `delay()` |
| [platforms/teensy40/errors.hpp](platforms/teensy40/errors.hpp) | `#include <Arduino.h>`, `TeensyTimerTool::errorCode`, `Serial.*`, `exit()` |
| [platforms/teensy40/demos/](platforms/teensy40/demos/) | Full Teensy hardware instantiation, hardcoded pins |

---

## Next Steps

### 1. Extract abstract interfaces into `core/`

`brushless_controller.hpp` currently depends on concrete Teensy types. The interfaces need to be pulled into `core/` as pure abstract base classes so the controller can be compiled without any hardware headers:

- `AbsoluteEncoder` (currently defined in `encoder.hpp` with Teensy includes) → `core/encoder_interface.hpp`
- `BrushlessDriver` → `core/driver_interface.hpp`
- `CurrentSensor` / `InlineCurrentSensorPackage` → `core/current_sensor_interface.hpp`

The Teensy concrete implementations (`SpiEncoder`, `BrushlessDriver`, `InlineCurrentSensor`) stay in `platforms/teensy40/` and inherit from these interfaces.

### 2. Refactor `brushless_controller.hpp` → `core/`

Once the interfaces are in `core/`, `BrushlessController` can move there. The remaining Teensy-specific pieces (timer callback, `delay()`, `Serial`) move to a thin platform shim in `platforms/teensy40/`.

### 3. Refactor `telemetry.hpp` → `core/`

Inject clock and output stream; remove Arduino dependency.

### 4. Write unit tests for `discrete_filter.hpp` and `motors.hpp`

These are already in `core/` but have no tests yet.

---

## Incomplete / Deferred Files

| File | Status |
|------|--------|
| `platforms/teensy40/brushed_controller.hpp` | Fully commented out; not yet implemented |
| `platforms/teensy40/chirp_controller.hpp` | Fully commented out; experimental |
