/**
 * @file adc_dma_hal.hpp
 * @brief iMXRT1062 ADC hardware-trigger + XBAR1 + DMA hardware abstraction layer.
 *
 * @details Provides functions to configure the iMXRT1062 ADC in hardware-trigger
 * mode, route a FlexPWM output trigger through XBAR1 and ADC_ETC to the ADC,
 * and set up DMA channels to move ADC results to memory without CPU involvement.
 *
 * ## Signal chain
 * @code
 * FlexPWM counter TOP
 *   → TCTRL trigger pulse (pwm_hal::configure_trigger_output)
 *   → XBARA1 routing (connect_flexpwm_trigger_to_adcN)
 *   → ADC_ETC trigger chain (configure_adc_etc_triggerN)
 *   → ADCn hardware conversion (configure_adcN_hardware_trigger)
 *   → DMA transfer on COCO (configure_dma_for_adcN)
 *   → volatile uint16_t buffer (read by InlineCurrentSensor::read_dma)
 * @endcode
 *
 * ## ADC_ETC
 * On iMXRT1062, the ADC External Trigger Controller (ADC_ETC) sits between
 * XBAR1 and the ADC. It must be enabled and configured with a trigger chain
 * before hardware-triggered ADC conversions will work. `configure_adc_etc_trigger1()`
 * and `configure_adc_etc_trigger2()` handle this for ADC1 and ADC2 respectively.
 *
 * ## Two-ADC approach
 * The current sensor package (Current_Sensors2) uses two sensors on A4 and A5,
 * which both map to ADC1 channels. For simultaneous sampling, this HAL implements:
 * - ADC1 → converts A4 (channel 11) via ADC_ETC chain, DMA channel 1
 * - ADC2 → converts A5 (channel 12) via ADC_ETC chain, DMA channel 2
 * Both are triggered by the same XBAR output.
 *
 * @warning After `configure_adcN_hardware_trigger()` is called, do NOT call
 * `analogRead()` on the DMA-managed pins. `analogRead()` resets `ADC_SC2`
 * to software-trigger mode, silently disabling the DMA trigger chain. All
 * `validate_offset()` (which uses `analogRead()`) calls must complete before
 * this function is called.
 *
 * @note All channel numbers in this file are iMXRT1062 ADC channel indices,
 * NOT Arduino analog pin aliases. Use `arduino_pin_to_adc1_channel()` to
 * convert. Channel-to-pin mapping must be verified against RM §30 Table 30-1.
 *
 * @todo Verify exact XBARA1_IN_* and XBARA1_OUT_* constant values against the
 * Teensyduino version installed. Constant names may differ between Teensyduino
 * releases. Check `<imxrt.h>` and `<xbar.h>` for the installed version.
 *
 * @todo Consider using a single ADC1 with ADC_ETC chain-of-2 to avoid needing
 * ADC2 entirely, at the cost of sequential (not simultaneous) sampling of the
 * two current sensor channels.
 *
 * @see pwm_hal.hpp for FlexPWMPin definition and trigger output configuration.
 * @see current_sense.hpp for how the DMA buffers are consumed.
 */

#ifndef ADC_DMA_HAL_HPP
#define ADC_DMA_HAL_HPP

#include <Arduino.h>
#include <imxrt.h>
#include <DMAChannel.h>
#include "pwm_hal.hpp"

// xbar_connect is defined in pwm.c (Teensyduino core) with no public header.
extern "C" void xbar_connect(unsigned int input, unsigned int output);

namespace adc_dma_hal {

// ---------------------------------------------------------------------------
// Pin → ADC channel mapping
// ---------------------------------------------------------------------------

/**
 * @brief Maps an Arduino analog pin alias to its iMXRT1062 ADC1 channel number.
 *
 * @details The iMXRT1062 ADC uses hardware channel indices that differ from the
 * Arduino `A0`–`A9` aliases. This function provides the lookup needed to
 * configure the `ADC_HC0` register for hardware-triggered conversions.
 *
 * Mapping for Teensy 4.0 / 4.1 (verify against iMXRT1062 RM §30, Table 30-1):
 *
 * | Arduino | GPIO pad            | ADC1 ch | ADC2 ch |
 * |---------|---------------------|---------|---------|
 * | A0 (14) | GPIO_AD_B1_02       | 7       | —       |
 * | A1 (15) | GPIO_AD_B1_03       | 8       | —       |
 * | A2 (16) | GPIO_AD_B1_04       | 9       | —       |
 * | A3 (17) | GPIO_AD_B1_05       | 10      | —       |
 * | A4 (18) | GPIO_AD_B1_06       | 11      | —       |
 * | A5 (19) | GPIO_AD_B1_07       | 12      | —       |
 * | A6 (20) | GPIO_AD_B1_08       | 13      | —       |
 * | A7 (21) | GPIO_AD_B1_09       | 14      | —       |
 * | A8 (22) | GPIO_AD_B1_10       | 15      | —       |
 * | A9 (23) | GPIO_AD_B1_11       | —       | 0       |
 *
 * @param arduino_pin Arduino analog pin alias (e.g., `A4` = 18 on Teensy 4.x).
 * @returns iMXRT1062 ADC1 channel index [0, 15], or `255` if the pin is not
 *          an ADC1-capable input.
 *
 * @warning These mappings require hardware verification. Pin-to-channel
 * assignments can vary between Teensy board revisions.
 */
inline uint8_t arduino_pin_to_adc1_channel(uint8_t arduino_pin)
{
    switch (arduino_pin) {
        case 14: return  7;  // A0
        case 15: return  8;  // A1
        case 16: return  9;  // A2
        case 17: return 10;  // A3
        case 18: return 11;  // A4 — used by Current_Sensors2 sensor 0
        case 19: return 12;  // A5 — used by Current_Sensors2 sensor 1
        case 20: return 13;  // A6
        case 21: return 14;  // A7
        case 22: return 15;  // A8
        default: return 255; // Not an ADC1 channel or unrecognized
    }
}

/**
 * @brief Maps an Arduino analog pin alias to its iMXRT1062 ADC2 channel number.
 *
 * @param arduino_pin Arduino analog pin alias.
 * @returns iMXRT1062 ADC2 channel index [0, 15], or `255` if not an ADC2 input.
 */
inline uint8_t arduino_pin_to_adc2_channel(uint8_t arduino_pin)
{
    switch (arduino_pin) {
        case 23: return  0;  // A9 — GPIO_AD_B1_11, ADC2 channel 0
        default: return 255;
    }
}

// ---------------------------------------------------------------------------
// XBAR1 routing
// ---------------------------------------------------------------------------

/**
 * @brief Routes a FlexPWM submodule output trigger to ADC1 ETC trigger 0 via XBAR1.
 *
 * @details Enables the XBAR1 clock (`CCM_CCGR2`) and calls `xbar_connect()` to
 * connect the FlexPWM trigger output to the ADC_ETC trigger-0 input for ADC1.
 *
 * The XBARA1 routing used:
 * @code
 * XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG0  →  XBARA1_OUT_ADC1_ETC_TRIG00
 * @endcode
 *
 * This function currently hard-codes the trigger source to FLEXPWM2 SM[2]
 * (pin 9 on GateDriver2). A future improvement would dynamically select the
 * XBARA1 input based on the `trigger_source` argument.
 *
 * @param trigger_source FlexPWMPin from `BrushlessDriver::get_trigger_source()`.
 *                       Currently only FLEXPWM2 SM[2] is mapped; other
 *                       submodules will compile but may not route correctly.
 * @returns `true` on success, `false` if `trigger_source.module == nullptr`.
 *
 * @todo Implement a full XBARA1_IN lookup table keyed on (module, submodule)
 *       to support GateDriver1 (pins 3,4,5) and GateDriver3 (pins 29,33,39).
 *
 * @todo Verify XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG0 and XBARA1_OUT_ADC1_ETC_TRIG00
 *       constant names against the Teensyduino version installed. These constants
 *       are defined in `<imxrt.h>` and may be named differently in older releases.
 */
inline bool connect_flexpwm_trigger_to_adc1(const pwm_hal::FlexPWMPin & trigger_source)
{
    if (trigger_source.module == nullptr) { return false; }

    // Enable XBAR1 peripheral clock before any XBAR register access
    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);

    // Route FlexPWM2 SM[2] output trigger → ADC_ETC trigger 0 for ADC1.
    // xbar_connect() writes the XBARA1_SEL register for this output.
    // Constant names verified against Teensyduino 1.58 imxrt.h.
    xbar_connect(XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00);

    return true;
}

/**
 * @brief Routes a FlexPWM submodule output trigger to a specific ADC1 ETC trigger
 *        index via XBAR1.
 *
 * @details Routes the same FlexPWM trigger edge to `ADC1_ETC_TRIG0{trig_idx}`,
 * allowing up to three sensors on ADC1 to be triggered simultaneously (sequentially
 * in hardware) by a single PWM counter-TOP event. Each sensor uses its own
 * ADC_ETC trigger index (0, 1, or 2), which controls an independent ADC1 channel
 * conversion and DMA transfer.
 *
 * | `trig_idx` | XBAR output              |
 * |-----------|--------------------------|
 * | 0         | `XBARA1_OUT_ADC1_ETC_TRIG00` |
 * | 1         | `XBARA1_OUT_ADC1_ETC_TRIG01` |
 * | 2         | `XBARA1_OUT_ADC1_ETC_TRIG02` |
 *
 * @param trigger_source FlexPWMPin from `BrushlessDriver::get_trigger_source()`.
 * @param trig_idx       ADC_ETC trigger index [0, 2]. Matches the sensor position
 *                       within the `InlineCurrentSensorPackage` (sensor 0 → TRIG0,
 *                       sensor 1 → TRIG1, sensor 2 → TRIG2).
 * @returns `true` on success, `false` if `trigger_source.module == nullptr` or
 *          `trig_idx > 2`.
 *
 * @todo Verify XBARA1_OUT_ADC1_ETC_TRIG01 and XBARA1_OUT_ADC1_ETC_TRIG02 constant
 *       names against the installed Teensyduino version. They may differ from the
 *       names used here. Check `<imxrt.h>` or `<xbar.h>`.
 */
inline bool connect_flexpwm_trigger_to_adc1_trig(
    const pwm_hal::FlexPWMPin & trigger_source, uint8_t trig_idx)
{
    if (trigger_source.module == nullptr) { return false; }

    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);

    switch (trig_idx) {
        case 0:
            xbar_connect(XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00);
            break;
        case 1:
            xbar_connect(XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG01);
            break;
        case 2:
            xbar_connect(XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG02);
            break;
        default:
            return false;
    }
    return true;
}

/**
 * @brief Routes a FlexPWM submodule output trigger to ADC2 via XBAR1 and ADC_ETC.
 *
 * @details Mirrors `connect_flexpwm_trigger_to_adc1()` for ADC2. Both functions
 * can be called with the same `trigger_source`, causing a single FlexPWM pulse
 * to simultaneously trigger both ADC1 and ADC2 conversions.
 *
 * @param trigger_source FlexPWMPin from `BrushlessDriver::get_trigger_source()`.
 * @returns `true` on success, `false` if `trigger_source.module == nullptr`.
 */
inline bool connect_flexpwm_trigger_to_adc2(const pwm_hal::FlexPWMPin & trigger_source)
{
    if (trigger_source.module == nullptr) { return false; }

    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);

    // ADC2 is routed through ADC_ETC TRIG1 (configured in configure_adc_etc_trigger2)
    xbar_connect(XBARA1_IN_FLEXPWM2_PWM2_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG01);

    return true;
}

// ---------------------------------------------------------------------------
// ADC_ETC configuration
// ---------------------------------------------------------------------------

/**
 * @brief Configures a single ADC_ETC trigger index for one ADC1 channel conversion.
 *
 * @details Enables ADC_ETC and programs `TRIG[trig_idx]` with a single-entry chain
 * that converts `adc_channel` on ADC1 and issues a DMA request on completion.
 * Multiple sensors in the same package are each assigned a unique `trig_idx` so
 * that a single FlexPWM PWM counter-TOP edge causes each ADC_ETC trigger to fire
 * independently, each controlling one channel.
 *
 * ADC_ETC trigger 0 routes to ADC1, triggers 1 and 2 also route to ADC1 (via
 * separate XBAR connections made by `connect_flexpwm_trigger_to_adc1_trig()`).
 * ADC1 processes the conversions sequentially since there is only one ADC1 core;
 * the inter-trigger latency is approximately one ADC conversion time (~0.5 µs at
 * 12-bit resolution with ADLSMP).
 *
 * | `trig_idx` | ADC_ETC register pair              | DMAMUX source (approx.)       |
 * |-----------|-------------------------------------|-------------------------------|
 * | 0         | `ADC_ETC_TRIG0_CTRL / CHAIN_1_0`   | `DMAMUX_SOURCE_ADC_ETC`       |
 * | 1         | `ADC_ETC_TRIG1_CTRL / CHAIN_1_0`   | `DMAMUX_SOURCE_ADC_ETC + 1`   |
 * | 2         | `ADC_ETC_TRIG2_CTRL / CHAIN_1_0`   | `DMAMUX_SOURCE_ADC_ETC + 2`   |
 *
 * @param trig_idx   ADC_ETC trigger index [0, 2]. Matches `InlineCurrentSensor`
 *                   position within `InlineCurrentSensorPackage`.
 * @param adc_channel iMXRT1062 ADC1 channel number (NOT Arduino pin alias).
 *                    Use `arduino_pin_to_adc1_channel()` to convert.
 *
 * @note Must be called after `connect_flexpwm_trigger_to_adc1_trig()` routes the
 *       XBAR output for this trigger index.
 *
 * @todo Verify ADC_ETC_TRIG2_CTRL and ADC_ETC_TRIG2_CHAIN_1_0 constant names
 *       exist in the installed Teensyduino imxrt.h.
 * @todo Verify DMAMUX source offsets: `DMAMUX_SOURCE_ADC_ETC + trig_idx` assumes
 *       consecutive DMAMUX request lines per trigger. Check RM §4 Table 4-3.
 */
inline void configure_adc_etc_trig(uint8_t trig_idx, uint8_t adc_channel)
{
    // ADC_ETC is always clocked on iMXRT1062; no separate CCM gate needed.
    ADC_ETC_CTRL &= ~ADC_ETC_CTRL_SOFTRST;
    // Enable the bit corresponding to this trigger index
    ADC_ETC_CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(1u << trig_idx);

    const uint32_t chain_entry =
        ADC_ETC_TRIG_CHAIN_CSEL0(adc_channel)   |  // ADC channel selection
        ADC_ETC_TRIG_CHAIN_HWTS0(1u)            |  // hardware trigger select
        ADC_ETC_TRIG_CHAIN_IE0(0b01u)           |  // DMA request on conversion done
        ADC_ETC_TRIG_CHAIN_B2B0;                   // back-to-back trigger enable

    switch (trig_idx) {
        case 0:
            ADC_ETC_TRIG0_CTRL     = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(0u);
            ADC_ETC_TRIG0_CHAIN_1_0 = chain_entry;
            break;
        case 1:
            ADC_ETC_TRIG1_CTRL     = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(0u);
            ADC_ETC_TRIG1_CHAIN_1_0 = chain_entry;
            break;
        case 2:
            ADC_ETC_TRIG2_CTRL     = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(0u);
            ADC_ETC_TRIG2_CHAIN_1_0 = chain_entry;
            break;
        default:
            break;
    }
}

/**
 * @brief Legacy wrapper: configures ADC_ETC TRIG0 for one ADC1 channel.
 * @deprecated Prefer `configure_adc_etc_trig(0, adc_channel)`.
 * @param adc_channel iMXRT1062 ADC1 channel number.
 */
inline void configure_adc_etc_trigger1(uint8_t adc_channel)
{
    configure_adc_etc_trig(0, adc_channel);
}

/**
 * @brief Configures ADC_ETC TRIG1 for one ADC2 channel.
 *
 * @details Used for pins that map exclusively to ADC2 (e.g., A9 = GPIO_AD_B1_11
 * = ADC2 channel 0). TRIG1 is conventionally assigned to ADC2 in dual-ADC
 * configurations.
 *
 * @param adc_channel iMXRT1062 ADC2 channel number.
 *                    Use `arduino_pin_to_adc2_channel()` to convert.
 */
inline void configure_adc_etc_trigger2(uint8_t adc_channel)
{
    // ADC_ETC is always clocked on iMXRT1062; no separate CCM gate needed.
    ADC_ETC_CTRL &= ~ADC_ETC_CTRL_SOFTRST;
    ADC_ETC_CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(0b11u); // TRIG0 and TRIG1

    ADC_ETC_TRIG1_CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(0u);

    ADC_ETC_TRIG1_CHAIN_1_0 =
        ADC_ETC_TRIG_CHAIN_CSEL0(adc_channel)   |
        ADC_ETC_TRIG_CHAIN_HWTS0(1u)            |
        ADC_ETC_TRIG_CHAIN_IE0(0b01u)           |
        ADC_ETC_TRIG_CHAIN_B2B0;
}

// ---------------------------------------------------------------------------
// ADC hardware-trigger configuration
// ---------------------------------------------------------------------------

/**
 * @brief Configures ADC1 for hardware-trigger mode on a specified channel.
 *
 * @details Sets the following ADC1 registers:
 * - `ADC1_CFG`: 12-bit resolution, low-speed conversion (ADLSMP=1 for longer
 *   sample time), no averaging (oversampling is handled in software via filters).
 * - `ADC1_SC2`: `ADTRG=1` — hardware trigger mode. **Critical**: after this
 *   call, any `analogRead()` on ADC1 channels will reset ADTRG to 0 (software
 *   trigger), silently breaking the DMA trigger chain.
 * - `ADC1_HC0`: set to `adc_channel`, selecting the analog input for the next
 *   hardware-triggered conversion.
 *
 * @param adc_channel iMXRT1062 ADC1 channel number.
 * @returns `true` always (no failure detection in hardware register writes).
 *
 * @warning Must be called **after** all `InlineCurrentSensor::init_sensor()`
 *          calls, which use `analogRead()` (software trigger) internally via
 *          `validate_offset()`. Calling this function first and then calling
 *          `init_sensor()` will break the DMA trigger path.
 */
inline bool configure_adc1_hardware_trigger(uint8_t adc_channel)
{
    // Enable ADC1 clock (already on by default after analogRead(), but ensure it)
    CCM_CCGR1 |= CCM_CCGR1_ADC1(CCM_CCGR_ON);

    // 12-bit resolution, long sample time, no hardware averaging
    ADC1_CFG = ADC_CFG_ADICLK(1)    |  // ADCK = IPG clock / 2
               ADC_CFG_MODE(2)       |  // 12-bit (single-ended)
               ADC_CFG_ADLSMP;         // Long sample time (better noise rejection)

    // Hardware trigger mode — ADTRG=1 in GC register
    ADC1_CFG |= ADC_CFG_ADTRG;  // Hardware trigger mode (bit 13 of CFG register)

    // Select analog channel for hardware-triggered conversion
    ADC1_HC0 = adc_channel;

    return true;
}

/**
 * @brief Configures ADC2 for hardware-trigger mode on a specified channel.
 *
 * @details Mirrors `configure_adc1_hardware_trigger()` for ADC2 registers.
 *
 * @param adc_channel iMXRT1062 ADC2 channel number.
 * @returns `true` always.
 *
 * @warning Same restriction as ADC1: must be called after all `analogRead()` calls.
 */
inline bool configure_adc2_hardware_trigger(uint8_t adc_channel)
{
    CCM_CCGR1 |= CCM_CCGR1_ADC2(CCM_CCGR_ON);

    ADC2_CFG = ADC_CFG_ADICLK(1)    |
               ADC_CFG_MODE(2)       |
               ADC_CFG_ADLSMP;

    ADC2_CFG |= ADC_CFG_ADTRG;  // Hardware trigger mode (bit 13 of CFG register)

    ADC2_HC0 = adc_channel;

    return true;
}

// ---------------------------------------------------------------------------
// DMA channel configuration
// ---------------------------------------------------------------------------

/**
 * @brief Configures a DMA channel to transfer ADC1 results to a memory buffer.
 *
 * @details Sets up the DMA channel to:
 * 1. Read a 16-bit value from `ADC1_R0` (the ADC1 result register) each time
 *    the DMAMUX source (`DMAMUX_SOURCE_ADC_ETC`) fires.
 * 2. Write it to `dest_buf`.
 * 3. Transfer 2 bytes (16-bit word) per trigger.
 * 4. Auto-reload after each transfer (persistent, fires on every ADC completion).
 *
 * After this call, `dest_buf` will be updated automatically each time a
 * hardware-triggered ADC1 conversion completes — no CPU involvement.
 *
 * @param ch       DMA channel object (must be a persistent/global object, not stack).
 *                 `DMAChannel` is from Teensyduino's `<DMAChannel.h>`.
 * @param dest_buf Destination buffer (must be `volatile`, must have persistent
 *                 lifetime — global or static). Typically a private member of
 *                 `InlineCurrentSensor`.
 * @returns `true` always.
 *
 * @note `DMAMUX_SOURCE_ADC_ETC` triggers the DMA on ADC_ETC conversion done,
 * which is more reliable than `DMAMUX_SOURCE_ADC1` when ADC_ETC is in use.
 * If ADC_ETC is bypassed in a future revision, switch to `DMAMUX_SOURCE_ADC1`.
 *
 * @todo Verify that `DMAMUX_SOURCE_ADC_ETC` is the correct DMA MUX source for
 * ADC1-via-ADC_ETC on iMXRT1062. Refer to RM §4 (DMAMUX) Table 4-3 for the
 * complete DMA request source table.
 */
inline bool configure_dma_for_adc1(DMAChannel & ch, volatile uint16_t & dest_buf)
{
    ch.begin();
    // ADC1_R0 is a 32-bit register; the result occupies the low 16 bits at the same address.
    // The cast is intentional hardware register access — suppress the strict-aliasing warning.
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstrict-aliasing"
    ch.source(*reinterpret_cast<volatile uint16_t *>(&ADC1_R0));  // Source: ADC1 result register (low 16 bits)
    #pragma GCC diagnostic pop
    ch.destination(dest_buf);                    // Destination: caller's buffer
    ch.transferSize(2);                          // 16-bit transfer
    ch.transferCount(1);                         // 1 transfer per trigger

    // Trigger on ADC_ETC conversion-done DMA request
    ch.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC_ETC);

    ch.enable();
    return true;
}

/**
 * @brief Configures a DMA channel to transfer ADC2 results to a memory buffer.
 *
 * @details Mirrors `configure_dma_for_adc1()` for ADC2. Uses `ADC2_R0` as the
 * source register. Ensure `configure_adc2_hardware_trigger()` and
 * `configure_adc_etc_trigger2()` have been called first.
 *
 * @param ch       Persistent DMA channel object.
 * @param dest_buf Persistent volatile destination buffer.
 * @returns `true` always.
 */
inline bool configure_dma_for_adc2(DMAChannel & ch, volatile uint16_t & dest_buf)
{
    ch.begin();
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstrict-aliasing"
    ch.source(*reinterpret_cast<volatile uint16_t *>(&ADC2_R0));
    #pragma GCC diagnostic pop
    ch.destination(dest_buf);
    ch.transferSize(2);
    ch.transferCount(1);
    ch.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC_ETC);
    ch.enable();
    return true;
}

/**
 * @brief Configures a DMA channel for ADC1 results triggered by a specific
 *        ADC_ETC trigger index.
 *
 * @details Each sensor in a package is assigned a unique ADC_ETC trigger index
 * (`trig_idx`). This function programs the DMA channel to fire on completion of
 * the corresponding ADC_ETC trigger's chain entry 0, reading from `ADC1_R0` and
 * storing to `dest_buf`.
 *
 * The DMAMUX source used is `DMAMUX_SOURCE_ADC_ETC + trig_idx`:
 * - `trig_idx = 0`: `DMAMUX_SOURCE_ADC_ETC + 0` (same as existing TRIG0 path)
 * - `trig_idx = 1`: `DMAMUX_SOURCE_ADC_ETC + 1`
 * - `trig_idx = 2`: `DMAMUX_SOURCE_ADC_ETC + 2`
 *
 * @param ch       Persistent `DMAChannel` object (global or static member).
 * @param dest_buf Persistent `volatile uint16_t` destination (global or static).
 * @param trig_idx ADC_ETC trigger index [0, 2] matching the sensor position in
 *                 the `InlineCurrentSensorPackage`.
 * @returns `true` always.
 *
 * @warning `ch` and `dest_buf` must outlive the control loop. Storing them on
 *          the stack will cause undefined behavior when the DMA fires after the
 *          stack frame is gone.
 *
 * @todo Verify that `DMAMUX_SOURCE_ADC_ETC + trig_idx` (sources 88, 89, 90) are
 *       the correct DMAMUX request lines for ADC_ETC TRIG0, TRIG1, TRIG2 on ADC1.
 *       Consult iMXRT1062 RM §4 Table 4-3 and confirm with Teensyduino `<imxrt.h>`.
 */
inline bool configure_dma_for_adc1_trig(
    DMAChannel & ch, volatile uint16_t & dest_buf, uint8_t trig_idx)
{
    ch.begin();
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wstrict-aliasing"
    ch.source(*reinterpret_cast<volatile uint16_t *>(&ADC1_R0));
    #pragma GCC diagnostic pop
    ch.destination(dest_buf);
    ch.transferSize(2);
    ch.transferCount(1);
    // DMAMUX source for ADC_ETC TRIG[trig_idx] chain0 done on ADC1.
    // @todo Verify offset: assumes consecutive request lines starting at DMAMUX_SOURCE_ADC_ETC.
    ch.triggerAtHardwareEvent(static_cast<uint8_t>(DMAMUX_SOURCE_ADC_ETC + trig_idx));
    ch.enable();
    return true;
}

} // namespace adc_dma_hal

#endif // ADC_DMA_HAL_HPP
