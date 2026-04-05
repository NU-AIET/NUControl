/**
 * @file nu_control.hpp
 * @brief Monolithic include header for the NUControl FOC library.
 *
 * @details Including this single header pulls in the complete NUControl stack
 * in dependency order:
 *
 * | Header                  | Provides |
 * |-------------------------|----------|
 * `errors.hpp`              | `WarningCodes`, `ErrorCodes`, `raise_warning()`, `raise_error()` |
 * `helpers.hpp`             | Math constants, `normalize_angle()`, `near_zero()`, `nu_sincos()` |
 * `motors.hpp`              | `MotorParameters`, `EC45_Flat`, `U2535` instances |
 * `discrete_filter.hpp`     | `DiscreteFilter<T,G>`, `Butterworth2nd<T>`, `PIController<T>` |
 * `vel_filters.hpp`         | `vel_filter_200_` 200-tap FIR velocity estimator |
 * `transformations.hpp`     | `PhaseValues<T>`, `AlphaBetaValues<T>`, `QuadDirectValues<T>`, Clarke/Park |
 * `encoder.hpp`             | `Angle`, `AbsoluteEncoder` |
 * `spi_encoder.hpp`         | `SPIEncoder` |
 * `pwm_hal.hpp`             | `FlexPWMPin`, center-aligned PWM HAL functions |
 * `adc_dma_hal.hpp`         | XBAR1 routing, ADC hardware trigger, DMA channel setup |
 * `current_sense.hpp`       | `InlineCurrentSensor`, `InlineCurrentSensorPackage` |
 * `driver.hpp`              | `BrushlessDriver`, `BrushedDriver` |
 * `anticog_helpers.hpp`     | `AnticoggingCompensator<steps_>` |
 * `brushless_controller.hpp`| `BrushlessController` |
 * `cogging_mapper.hpp`      | `CoggingMapper<steps_>` |
 * `userConfig.h`            | Project-specific pin/parameter overrides |
 *
 * @note `userConfig.h` is expected to be provided by the application project
 *       (not part of the NUControl library). It may define pin mappings, motor
 *       selection, and other application-specific constants that override
 *       defaults in the library headers.
 */

#ifndef NU_CONTROL
#define NU_CONTROL

#include "thermal_model.hpp"
#include "current_sense.hpp"
#include "discrete_filter.hpp"
#include "driver.hpp"
#include "errors.hpp"
#include "helpers.hpp"
#include "motors.hpp"
#include "brushless_controller.hpp"
#include "spi_encoder.hpp"
#include "transformations.hpp"
#include "cogging_mapper.hpp"
#include "anticog_helpers.hpp"
#include "pwm_hal.hpp"
#include "adc_dma_hal.hpp"
#include "userConfig.h"
#include "vel_filters.hpp"

#endif // NU_CONTROL
