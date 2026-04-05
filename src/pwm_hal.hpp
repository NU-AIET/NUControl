/**
 * @file pwm_hal.hpp
 * @brief iMXRT1062 FlexPWM hardware abstraction layer for center-aligned PWM.
 *
 * @details Provides a pin-to-FlexPWM lookup table and configuration functions to
 * switch the Teensy 4.x FlexPWM peripheral from edge-aligned (sawtooth) counting
 * to center-aligned (up-down / triangle) counting. Center-aligned mode creates a
 * deterministic quiet window at the counter peak where all phases are in a known
 * state — the optimal moment to trigger ADC current-sensor reads.
 *
 * ## Calling order
 * All functions here must be called **after** `analogWriteFrequency()` and
 * `analogWrite()` have already configured the FlexPWM peripheral, because those
 * calls establish `VAL1` (the counter modulus) and the initial duty values (`VAL3`,
 * `VAL5`) that `configure_center_aligned()` then adjusts.
 *
 * ## Center-aligned PWM mechanics
 * In edge-aligned mode (FULL=0): counter 0 → VAL1 → 0 (sawtooth), period = VAL1.
 * In center-aligned mode (FULL=1): counter 0 → VAL1 → 0 (triangle), period = 2×VAL1.
 *
 * `configure_center_aligned()` halves VAL1 to preserve the original PWM frequency
 * and scales VAL3/VAL5 accordingly. The VAL0 compare value is then set equal to
 * the new VAL1 so that the output trigger fires exactly at the counter peak.
 *
 * ## Hardware
 * Target: Teensy 4.0 / 4.1 (NXP iMXRT1062, ARM Cortex-M7, 150 MHz FlexPWM clock).
 *
 * @note The pin-to-FlexPWM table covers all Teensy 4.x PWM-capable pins. Entries
 * marked with `nullptr` module are non-PWM pins or mappings that require hardware
 * verification against the iMXRT1062 Reference Manual (RM Rev 3, §27).
 */

#ifndef PWM_HAL_HPP
#define PWM_HAL_HPP

#include <Arduino.h>
#include <imxrt.h>

namespace pwm_hal {

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/**
 * @brief Identifies a single FlexPWM output pin by its peripheral location.
 *
 * @details Stores a pointer to the FlexPWM module base address, the submodule
 * index (0–3), and the output channel within that submodule (A=0, B=1).
 * A null `module` pointer indicates an invalid or unrecognized pin.
 *
 * @note On iMXRT1062, each FlexPWM module has 4 submodules, each with two
 * complementary outputs (A and B). The A channel uses VAL2/VAL3 compare
 * registers; the B channel uses VAL4/VAL5.
 */
struct FlexPWMPin {
    IMXRT_FLEXPWM_t * module;   ///< Pointer to IMXRT_FLEXPWM1..4; nullptr = invalid
    uint8_t           submodule; ///< Submodule index within the module [0, 3]
    uint8_t           channel;   ///< Output channel: 0 = A (VAL2/VAL3), 1 = B (VAL4/VAL5)
};

// ---------------------------------------------------------------------------
// Pin lookup table (Teensy 4.0 / 4.1)
// ---------------------------------------------------------------------------

/**
 * @brief Returns the FlexPWM descriptor for an Arduino digital pin number.
 *
 * @details Implements a static lookup table mapping Arduino pin numbers to their
 * iMXRT1062 FlexPWM module, submodule, and channel. Only PWM-capable pins have
 * valid entries; all others return `{nullptr, 0, 0}`.
 *
 * The table covers the subset of pins used by the NUControl gate drivers:
 * - GateDriver1: pins 3, 4, 5
 * - GateDriver2: pins 7, 8, 9
 * - GateDriver3: pins 29, 33, 39
 *
 * @note Entries tagged `@warning` require hardware verification against the
 * iMXRT1062 Reference Manual pin mux table (RM §10, Table 10-1).
 *
 * @param arduino_pin Arduino digital pin number (0–41 on Teensy 4.1).
 * @returns FlexPWMPin descriptor. Check `module != nullptr` before use.
 */
inline FlexPWMPin get_flexpwm_pin(uint8_t arduino_pin)
{
    // iMXRT1062 FlexPWM mapping for Teensy 4.0 / 4.1
    // Source: Teensy 4.x schematic, iMXRT1062 RM §10 pin mux table, and
    // Teensyduino cores/teensy4/pwm.c pin_info table.
    //
    // channel: 0 = A (PWM_A output, VAL2/VAL3 registers)
    //          1 = B (PWM_B output, VAL4/VAL5 registers)
    switch (arduino_pin) {
        // --- FLEXPWM4 ---
        case  0: return {&IMXRT_FLEXPWM4, 2, 0}; // FLEXPWM4_PWMA2  GPIO_AD_B1_02
        case  1: return {&IMXRT_FLEXPWM4, 2, 1}; // FLEXPWM4_PWMB2  GPIO_AD_B1_03
        case  2: return {&IMXRT_FLEXPWM4, 2, 1}; // FLEXPWM4_PWMB2  (shared, verify)
        case 22: return {&IMXRT_FLEXPWM4, 0, 0}; // FLEXPWM4_PWMA0  GPIO_AD_B1_08
        case 23: return {&IMXRT_FLEXPWM4, 1, 0}; // FLEXPWM4_PWMA1  GPIO_AD_B1_09

        // --- FLEXPWM1 ---
        case  3: return {&IMXRT_FLEXPWM1, 3, 0}; // FLEXPWM1_PWMA3  GPIO_EMC_05
        case  7: return {&IMXRT_FLEXPWM1, 1, 1}; // FLEXPWM1_PWMB1  GPIO_B1_01
        case  8: return {&IMXRT_FLEXPWM1, 3, 1}; // FLEXPWM1_PWMB3  GPIO_B1_00
        case 24: return {&IMXRT_FLEXPWM1, 2, 0}; // FLEXPWM1_PWMA2  GPIO_AD_B0_12
        case 38: return {&IMXRT_FLEXPWM1, 2, 1}; // FLEXPWM1_PWMB2  GPIO_AD_B0_11

        // --- FLEXPWM2 ---
        case  4: return {&IMXRT_FLEXPWM2, 0, 0}; // FLEXPWM2_PWMA0  GPIO_EMC_06
        case  5: return {&IMXRT_FLEXPWM2, 1, 0}; // FLEXPWM2_PWMA1  GPIO_EMC_08
        case  6: return {&IMXRT_FLEXPWM2, 2, 0}; // FLEXPWM2_PWMA2  GPIO_EMC_10
        case  9: return {&IMXRT_FLEXPWM2, 2, 1}; // FLEXPWM2_PWMB2  GPIO_B0_11
        case 33: return {&IMXRT_FLEXPWM2, 0, 1}; // FLEXPWM2_PWMB0  GPIO_EMC_07
        case 36: return {&IMXRT_FLEXPWM2, 3, 1}; // FLEXPWM2_PWMB3  GPIO_EMC_25 (verify)
        case 37: return {&IMXRT_FLEXPWM2, 3, 0}; // FLEXPWM2_PWMA3  GPIO_AD_B0_13 (verify)
        case 39: return {&IMXRT_FLEXPWM2, 3, 0}; // FLEXPWM2_PWMA3  shared with 37 (verify)

        // --- FLEXPWM3 ---
        case 28: return {&IMXRT_FLEXPWM3, 3, 1}; // FLEXPWM3_PWMB3  GPIO_EMC_31 (verify)
        case 29: return {&IMXRT_FLEXPWM3, 2, 1}; // FLEXPWM3_PWMB2  GPIO_EMC_30 (verify)

        default: return {nullptr, 0, 0};           // Non-PWM pin or unrecognized
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/**
 * @brief Switches a FlexPWM submodule to center-aligned (up-down) counting mode.
 *
 * @details Performs four register modifications on the targeted submodule:
 *
 * 1. **VAL1 halved**: `SM[n].VAL1 /= 2` — In center-aligned mode the counter
 *    period is `2×VAL1` clock cycles, so halving VAL1 restores the original
 *    PWM frequency that `analogWriteFrequency()` configured.
 *
 * 2. **Duty values scaled**: `VAL3 /= 2` and `VAL5 /= 2` — Proportionally
 *    reduces the current duty compare values to maintain the same duty percentage.
 *    Subsequent `analogWrite()` calls read the new VAL1 and scale correctly.
 *
 * 3. **FULL bit set**: `SM[n].CTRL |= FLEXPWM_SMCTRL_FULL` — Switches the
 *    counter from edge-aligned (sawtooth 0→VAL1) to center-aligned (triangle
 *    0→VAL1→0). The counter peak is now at the exact midpoint of each period.
 *
 * 4. **VAL0 = VAL1**: Sets VAL0 to the new VAL1 value so that the output
 *    trigger (enabled separately by `configure_trigger_output()`) fires exactly
 *    at the counter peak — the quiet window where no PWM switching occurs.
 *
 * 5. **LDOK written**: Latches all shadow registers into the active registers.
 *    Without this step, none of the VAL register writes take effect.
 *
 * @param pin FlexPWMPin descriptor from `get_flexpwm_pin()`. Must have `module != nullptr`.
 *
 * @warning Must be called **after** `analogWriteFrequency()` and at least one
 *          `analogWrite()` call, so that VAL1 and VAL3/VAL5 are already populated.
 *          Calling on an unconfigured submodule reads garbage values for VAL1.
 *
 * @note Does not configure cross-submodule SYNC. If multiple submodules on
 *       different modules are used together, they run with independent clocks
 *       (same frequency but potentially small phase offset at startup).
 *       For single-module setups (all three phases on the same FlexPWM module),
 *       use `configure_submodule_sync()` to slave them to a common master.
 */
inline void configure_center_aligned(FlexPWMPin pin)
{
    if (pin.module == nullptr) { return; }

    auto & sm = pin.module->SM[pin.submodule];

    // Step 1: halve VAL1 to preserve PWM frequency
    // Edge-aligned period = VAL1 clocks; center-aligned period = 2×VAL1 clocks.
    uint16_t half_val1 = sm.VAL1 / 2u;

    // Step 2: scale duty compare values proportionally
    // A channel uses VAL3; B channel uses VAL5. Both are scaled regardless of
    // which channel this pin is, since both channels share the same submodule.
    uint16_t val3_new = sm.VAL3 / 2u;
    uint16_t val5_new = sm.VAL5 / 2u;

    sm.VAL1 = half_val1;
    sm.VAL3 = val3_new;
    sm.VAL5 = val5_new;

    // Step 3: set VAL0 to the counter peak value (= new VAL1).
    // The output trigger (if enabled by configure_trigger_output) fires at
    // the CNT == VAL0 compare, which is now the exact counter peak.
    sm.VAL0 = half_val1;

    // Step 4: enable center-aligned (up-down) counting.
    // OR-in to preserve existing CTRL fields (prescaler, etc.) set by analogWriteFrequency().
    sm.CTRL |= FLEXPWM_SMCTRL_FULL;

    // Step 5: latch all shadow registers into active registers.
    pin.module->MCTRL |= FLEXPWM_MCTRL_LDOK(1u << pin.submodule);
}

/**
 * @brief Enables the VAL0 output trigger on a FlexPWM submodule.
 *
 * @details Sets bit 0 of the TCTRL register (`OUT_TRIG_EN[0]`), which enables
 * the external trigger pulse when the counter equals VAL0. After
 * `configure_center_aligned()` sets `VAL0 = VAL1` (the counter peak), this
 * trigger fires once per PWM period at the exact moment all phases are in the
 * "quiet" state — the optimal ADC sampling point.
 *
 * The trigger output is routed to the ADC via XBAR1 (see `adc_dma_hal.hpp`).
 * Only one submodule per motor should have this enabled; the chosen submodule
 * becomes the ADC trigger source for all current sensors on that motor.
 *
 * @param pin FlexPWMPin descriptor. Must have `module != nullptr`.
 *            Typically the phase-A pin (e.g., pin 9 for GateDriver2).
 *
 * @note Must be called after `configure_center_aligned()` on the same pin,
 *       since that call establishes VAL0. Enabling the trigger before setting
 *       VAL0 may fire at the wrong counter value.
 */
inline void configure_trigger_output(FlexPWMPin pin)
{
    if (pin.module == nullptr) { return; }

    auto & sm = pin.module->SM[pin.submodule];

    // OUT_TRIG_EN[0] = enable VAL0 compare output trigger
    sm.TCTRL |= FLEXPWM_SMTCTRL_OUT_TRIG_EN(1u);

    // Latch TCTRL change
    pin.module->MCTRL |= FLEXPWM_MCTRL_LDOK(1u << pin.submodule);
}

/**
 * @brief Configures a slave submodule to use the master submodule's clock and reload.
 *
 * @details On the iMXRT1062, submodules within the same FlexPWM module can share
 * counting phase by slaving their clock and reload signals to a designated master
 * submodule (typically SM[0] or the lowest-indexed submodule). This ensures all
 * configured submodules reach their counter peak at the same instant, maximising
 * the quiet window during which the ADC trigger fires.
 *
 * This function is only meaningful for submodules **within the same FlexPWM
 * module**. Cross-module synchronisation (e.g., FLEXPWM1 ↔ FLEXPWM2) requires
 * XBAR routing and is not implemented here.
 *
 * @param module     Pointer to the FlexPWM module containing both submodules.
 * @param master_sm  Submodule index that acts as the clock/reload source [0, 3].
 * @param slave_sm   Submodule index to be slaved to the master [0, 3].
 *
 * @note After calling this function, write `FLEXPWM_MCTRL_LDOK` to latch the
 *       CTRL2 change if the module is actively running.
 *
 * @todo For applications requiring strict three-phase synchronisation across
 *       different FlexPWM modules, implement XBAR-based inter-module SYNC routing
 *       using `XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG0` → `XBARA1_OUT_FLEXPWM1_SM0_EXTSYNC`.
 */
inline void configure_submodule_sync(
    IMXRT_FLEXPWM_t * module, uint8_t master_sm, uint8_t slave_sm)
{
    if (module == nullptr) { return; }

    auto & sm = module->SM[slave_sm];

    // CLK_SEL = 2 → use submodule 0's clock (or whichever master is SM[0]).
    // If master_sm != 0, this approach has limitations; the RM §27.3.4 details
    // that CLK_SEL=2 always uses SM[0], not an arbitrary master.
    // For multi-motor setups where the master is not SM[0], use XBAR routing instead.
    (void)master_sm; // Stored for documentation; CLK_SEL=2 always means SM[0]
    sm.CTRL2 = (sm.CTRL2 & ~FLEXPWM_SMCTRL2_CLK_SEL(3u)) | FLEXPWM_SMCTRL2_CLK_SEL(2u);

    // RELOAD_SEL: use master submodule's local SYNC signal to reload compare values
    sm.CTRL2 |= FLEXPWM_SMCTRL2_RELOAD_SEL;

    module->MCTRL |= FLEXPWM_MCTRL_LDOK(1u << slave_sm);
}

} // namespace pwm_hal

#endif // PWM_HAL_HPP
