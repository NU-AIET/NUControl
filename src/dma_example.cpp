/**
 * @file dma_example.cpp
 * @brief Single-motor FOC example using DMA for both current sensing and SPI encoder.
 *
 * @details Demonstrates the full DMA pipeline on a Teensy 4.x:
 *
 * - **Current sensing (DMA)**: Triggered automatically at PWM counter-TOP by the
 *   FlexPWM → XBAR1 → ADC_ETC → ADC → DMA chain.  `init_components()` configures
 *   this via `BrushlessDriver::configure_center_aligned()` + `configure_adc_trigger()`.
 *   After init, `update_sensors()` reads the ADC result from DMA buffers in ~200 ns
 *   instead of blocking on `analogRead()` (~3–5 µs).
 *
 * - **Encoder (DMA prefetch)**: `SPIEncoder::begin_dma()` hands the encoder a
 *   persistent `DMAChannel`. The control ISR then uses the prefetch pattern:
 *   ```
 *   end of ISR N:   encoder.trigger_async_read()   // kick off DMA transfer
 *   start of ISR N+1: update_sensors()             // reads cached DMA result instantly
 *   ```
 *   `SPIEncoder::read()` (called inside `update_sensors()`) returns the DMA result
 *   immediately when `DMA_RESULT_READY` is set, with no SPI wait.
 *
 * ## Hardware
 * | Component            | Connection                            |
 * |----------------------|---------------------------------------|
 * | Gate driver (phases) | Pins 3 (A), 4 (B), 5 (C)             |
 * | Gate driver enable   | Pin 2                                 |
 * | Current sensors      | A0 (phase proxy 0), A1 (phase proxy 1)|
 * | Encoder (SPI)        | SPI bus (LPSPI4), CS = pin 10         |
 * | Motor                | Maxon EC45 Flat (8 pole pairs)        |
 * | Supply voltage       | 24 V                                  |
 *
 * ## Initialisation order (critical)
 * 1. `encoder.begin_dma(encoder_dma_ch)`  — allocates DMA channel for encoder
 * 2. `controller.init_components()`       — driver init + center-aligned PWM +
 *                                            current-sensor offset calibration +
 *                                            ADC/DMA trigger chain
 * 3. `controller.align_sensors()`         — or `load_calibration()` with saved values
 * 4. `encoder.trigger_async_read()`       — prime the first prefetch before the
 *                                            timer ISR starts
 * 5. `timer_.begin(update, 100)`          — start 10 kHz control loop
 *
 * @warning Do NOT call `analogRead()` on A0 or A1 after `init_components()` returns.
 *          Doing so resets `ADC_SC2` to software-trigger mode and silently disables
 *          the DMA current-sense path.
 *
 * @warning `encoder_dma_ch` must have global (static) lifetime — `SPIEncoder` holds
 *          a pointer to it and the DMA ISR fires asynchronously.
 *
 * @warning Set the DMA channel interrupt priority ≥ the control ISR priority, or the
 *          encoder DMA state machine will stall mid-transfer.
 */

#include <Arduino.h>
#include <TeensyTimerTool.h>
#include <DMAChannel.h>
#include "nu_control.hpp"

// ---------------------------------------------------------------------------
// DMA channel for encoder async reads — must have global/static lifetime.
// SPIEncoder::begin_dma() stores a pointer to this object.
// ---------------------------------------------------------------------------
DMAChannel encoder_dma_ch;

// ---------------------------------------------------------------------------
// Hardware constants
// ---------------------------------------------------------------------------
constexpr float CURR_GAIN    = 5.f;    ///< Current sensor gain [A/V]
constexpr int   ADC_RES      = 10;     ///< ADC resolution [bits]
constexpr float PWM_FREQ     = 20000.f;///< Center-aligned PWM frequency [Hz]
constexpr int   PWM_RES      = 12;     ///< PWM duty-cycle resolution [bits]
constexpr float SUPPLY_VOLT  = 24.f;   ///< Bus voltage [V]

// ---------------------------------------------------------------------------
// Current sensors  (A0 → sensor 0, A1 → sensor 1)
// ---------------------------------------------------------------------------
InlineCurrentSensor Phase_A{A0, CURR_GAIN, ADC_RES};
InlineCurrentSensor Phase_B{A1, CURR_GAIN, ADC_RES};

InlineCurrentSensorPackage<2> Current_Sensors{{&Phase_A, &Phase_B}};

// ---------------------------------------------------------------------------
// SPI encoder  (AMS AS5047P / AS5147P on SPI / LPSPI4, CS = pin 10)
//
// ams5x47::make_read_cmd(REG_ANGLECOM) builds the 16-bit command with correct
// even parity.  The legacy constant (0b11 << 14) | 0x3FFF = 0xFFFF is
// equivalent but less self-documenting.
// ---------------------------------------------------------------------------
SPIEncoder Encoder{ams5x47::make_read_cmd(ams5x47::REG_ANGLECOM), SPI, 10};

// ---------------------------------------------------------------------------
// Gate driver  (phases A/B/C = pins 3/4/5, enable = pin 2)
// ---------------------------------------------------------------------------
BrushlessDriver GateDriver{{3, 4, 5}, 2, PWM_FREQ, PWM_RES, SUPPLY_VOLT};

// ---------------------------------------------------------------------------
// Controller  (EC45_Flat is declared in motors.hpp)
// ---------------------------------------------------------------------------
BrushlessController controller{EC45_Flat, GateDriver, Current_Sensors, Encoder};

// ---------------------------------------------------------------------------
// Timer
// ---------------------------------------------------------------------------
TeensyTimerTool::PeriodicTimer timer_(TeensyTimerTool::TCK);

// ---------------------------------------------------------------------------
// Control ISR — called every 100 µs (10 kHz)
//
// DMA prefetch pattern:
//   update_sensors()           reads the DMA encoder result cached by the
//                              trigger_async_read() at the end of ISR N-1,
//                              and reads current from ADC DMA buffers.
//   trigger_async_read()       kicks off the encoder SPI transfer for ISR N+1
//                              so the result is ready before the next call to
//                              update_sensors().
// ---------------------------------------------------------------------------
void update()
{
  // Consume DMA encoder result + DMA current samples (both non-blocking)
  controller.update_sensors();

  // --- Application control law goes here ---
  // Example: zero-torque hold
  controller.set_target(0.f);

  controller.update_control();

  // Prefetch encoder angle for the next ISR cycle via DMA
  // Encoder.trigger_async_read();
}

void test(){
  
  Serial.println(Encoder.read());
  // Encoder.trigger_async_read();


}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup()
{
  while (!Serial) {}
  Serial.println("DMA example starting");

  TeensyTimerTool::attachErrFunc(timer_errors);

  // 1. Arm encoder DMA channel (must precede init_components so the DMA path
  //    is ready before the first control tick fires).
  // if (!Encoder.begin_dma(encoder_dma_ch)) {
  //   Serial.println("ERROR: Encoder DMA init failed (unsupported SPI bus?)");
  //   exit(1);
  // }
  Serial.println("Encoder DMA armed");

  // 2. Initialise driver (center-aligned PWM) + current-sensor offset
  //    calibration + ADC/DMA trigger chain.
  //    Order matters: init_sensors() (analogRead offset cal) runs before
  //    init_dma_sensors() so the ADC is not reset to software-trigger mode.
  if (!controller.init_components()) {
    Serial.println("ERROR: Controller component init failed");
    exit(1);
  }
  Serial.println("Components initialised (PWM + ADC DMA active)");

  // 3. Automated sensor alignment (rotates motor briefly).
  //    Replace with controller.load_calibration(saved_calib) to skip rotation.
  Serial.println("Aligning sensors...");
  if (!controller.align_sensors()) {
    Serial.println("ERROR: Sensor alignment failed");
    exit(1);
  }
  Serial.println("Sensors aligned");
  controller.print_calibration();

  delay(1000);

  // 4. Configure controller
  controller.set_control_mode(ControllerMode::TORQUE);
  controller.set_target(0.f);
  controller.set_feedback_state(false);

  // 5. Prime the encoder DMA prefetch so the very first call to update_sensors()
  //    finds a DMA result ready rather than falling back to a blocking read.
  Encoder.trigger_async_read();

  // 6. Start control loop (100 µs period) and 10 kHz timer
  controller.start_control(100, false);
  timer_.begin(update, 100);

  // Serial.println("Control loop running");
}

void loop()
{
  // // ── DMA validation loop ──────────────────────────────────────────────────
  // // Do NOT call read() after trigger_async_read() — read() spin-waits until
  // // DMA_RESULT_READY, which hangs if the DMA ISR chain doesn't complete.
  // // Use dma_result_ready() + read_async() instead so the loop never blocks.

  // Encoder.trigger_async_read();

  // // Give the DMA ISR chain time to complete (two SPI frames at 10 MHz ≈ 4 µs).
  // // 1 ms is vastly more than enough; if result is still not ready, ISR failed.
  // constexpr uint32_t TIMEOUT_US = 1000;
  // uint32_t t0 = micros();
  // while (!Encoder.dma_result_ready() && (micros() - t0) < TIMEOUT_US) {}

  // if (Encoder.dma_result_ready()) {
  //   Serial.print("DMA angle: ");
  //   Serial.println(Encoder.read_async(), 4);
  // } else {
  //   Serial.print("DMA TIMEOUT — busy=");
  //   Serial.print(Encoder.dma_busy());
  //   Serial.print("  ready=");
  //   Serial.println(Encoder.dma_result_ready());

  //   // ── LPSPI4 register dump ─────────────────────────────────────────────
  //   uint32_t sr  = IMXRT_LPSPI4_S.SR;
  //   uint32_t fsr = IMXRT_LPSPI4_S.FSR;
  //   Serial.print("  LPSPI4 SR=0x");    Serial.print(sr,  HEX);
  //   Serial.print("  FSR=0x");          Serial.print(fsr, HEX);
  //   Serial.print("  TCF=");   Serial.print((sr >>  9) & 1);
  //   Serial.print("  RDF=");   Serial.print((sr >>  1) & 1);
  //   Serial.print("  RXCOUNT="); Serial.println((fsr >> 16) & 0xFF);

  //   // ── DMA TCD dump ─────────────────────────────────────────────────────
  //   // DMA drained the RX FIFO (RXCOUNT=0) but the completion ISR didn't fire.
  //   // Check whether INTMAJOR (TCD.CSR bit 1) is actually set when the DMA runs,
  //   // and whether the NVIC has the channel interrupt enabled.
  //   //
  //   // INTMAJOR=0 → attachInterrupt() didn't set it, or something cleared it after.
  //   //              Fix: call encoder_dma_ch.interruptAtCompletion() explicitly in
  //   //              begin_dma(), or check call order in _configure_dma_for_frame().
  //   // INTMAJOR=1, NVIC_EN=0 → NVIC_ENABLE_IRQ was not called for this channel.
  //   // INTMAJOR=1, NVIC_EN=1 → ISR fires but crashes silently, or wrong vector index.
  //   uint8_t  ch      = encoder_dma_ch.channel;
  //   uint16_t tcd_csr = encoder_dma_ch.TCD->CSR;
  //   uint8_t  intmajor = (tcd_csr >> 1) & 1;               // DMA_TCD_CSR_INTMAJOR
  //   uint8_t  nvic_en  = (ch < 32)
  //                       ? ((NVIC_ISER0 >> ch) & 1)         // channels 0-31
  //                       : ((NVIC_ISER1 >> (ch - 32)) & 1); // channels 32-63
  //   Serial.print("  DMA ch=");      Serial.print(ch);
  //   Serial.print("  TCD.CSR=0x");   Serial.print(tcd_csr, HEX);
  //   Serial.print("  INTMAJOR=");    Serial.print(intmajor);
  //   Serial.print("  NVIC_EN=");     Serial.println(nvic_en);
  // }

  // delay(10);
}
