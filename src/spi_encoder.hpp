/**
 * @file spi_encoder.hpp
 * @brief SPI absolute encoder driver for AMS AS5047P / AS5147P with DMA async reads
 *        and parity / error-flag validation.
 *
 * @details Implements `AbsoluteEncoder::read()` using the standard two-transaction
 * SPI protocol required by all AMS AS504x / AS514x devices:
 *
 * **Transaction 1 — Command frame** (latches the conversion):
 * ```
 * CS↓ → transfer16(read_cmd_) → CS↑ → delay 400 ns (tCSn hold)
 * ```
 * **Transaction 2 — Data frame** (returns the latched angle):
 * ```
 * CS↓ → transfer16(0x0000) → CS↑
 * ```
 *
 * ## Blocking vs. DMA async
 *
 * **Blocking mode** (default, backward-compatible): `read()` executes both frames
 * synchronously, consuming ≈3–4 µs at 10 MHz SPI.
 *
 * **DMA async / prefetch mode**: Call `begin_dma(channel)` once after construction,
 * then use the prefetch pattern inside the control ISR:
 * ```cpp
 * // ── End of ISR N ──────────────────────────────────────────
 * encoder.trigger_async_read();   // kick off frames 1 & 2 in background
 *
 * // ── Start of ISR N+1 ──────────────────────────────────────
 * float angle = encoder.read_async(); // instant — DMA already finished
 * ```
 * The two SPI frames run entirely via DMA and an interrupt-chained state machine
 * (LPSPI RX DMA → CS-toggle ISR → LPSPI RX DMA). The control ISR is not blocked
 * by any SPI wait; the angle read overlaps with the ADC DMA conversion.
 *
 * ## Validation pipeline (applied in order)
 * 1. **Even parity** (bit 15): `__builtin_popcount(frame) % 2 == 0`. Disable with
 *    `set_parity_check_state(false)`.
 * 2. **Error Flag** (bit 14): AS5047P sets EF=1 when a sensor fault is pending.
 *    Read `ams5x47::REG_ERRFL` to identify the specific fault. Disable with
 *    `set_error_flag_check_state(false)`.
 * 3. **Glitch filter**: rejects raw counts 0 and (2^res − 1), which are the
 *    saturation codes produced when the AGC hits its rails. Disable with
 *    `set_glitch_filter_state(false)`.
 *
 * Any invalid frame returns −1.0 from `read()` / `read_async()`. `Angle::update_angle()`
 * silently discards negative values.
 *
 * ## Platform
 * Targets **iMXRT1062 (Teensy 4.x)**. The DMA async path uses raw LPSPI peripheral
 * registers (`IMXRT_LPSPI4_S` etc.) and `DMAChannel` from `<DMAChannel.h>`. The
 * blocking path is portable to any Arduino-compatible SPI target.
 *
 * @note The SPI bus is opened with `beginTransaction()` in the constructor and held
 *       open permanently. `SPIEncoder` assumes exclusive ownership of the bus.
 *       If multiple devices share the bus, move `beginTransaction / endTransaction`
 *       into `read_raw()`.
 */

#ifndef SPI_ENCODER_HPP
#define SPI_ENCODER_HPP

#include "encoder.hpp"
#include <DMAChannel.h>


// ===========================================================================
// AMS AS5047P / AS5147P protocol constants
// ===========================================================================

/**
 * @brief Register map and frame-level utilities for the AMS AS5047P / AS5147P.
 *
 * @details Both devices share the same 16-bit SPI frame format:
 *
 * **Command frame** (host → sensor):
 * | Bit 15       | Bit 14        | Bits 13:0         |
 * |--------------|---------------|-------------------|
 * | Even parity  | R/W (1=read)  | 14-bit reg addr   |
 *
 * **Response frame** (sensor → host):
 * | Bit 15       | Bit 14        | Bits 13:0         |
 * |--------------|---------------|-------------------|
 * | Even parity  | EF (error)    | 14-bit data       |
 *
 * Parity is **even** over the full 16-bit word:
 * `__builtin_popcount(frame) & 1 == 0` must hold.
 */
namespace ams5x47 {

// ---------------------------------------------------------------------------
// Register addresses (14-bit)
// ---------------------------------------------------------------------------

constexpr uint16_t REG_ERRFL    = 0x0001u; ///< Error flags (SPI err, offset err, CORDIC err)
constexpr uint16_t REG_PROG     = 0x0003u; ///< OTP programming register
constexpr uint16_t REG_DIAAGC   = 0x3FFCu; ///< Diagnostics & automatic gain control value
constexpr uint16_t REG_MAG      = 0x3FFDu; ///< CORDIC magnitude (field strength indicator)
constexpr uint16_t REG_ANGLEUNC = 0x3FFEu; ///< Angle without dynamic error compensation
constexpr uint16_t REG_ANGLECOM = 0x3FFFu; ///< Angle with dynamic angle error compensation

// ---------------------------------------------------------------------------
// Frame field masks
// ---------------------------------------------------------------------------

constexpr uint16_t PARITY_MASK = 0x8000u; ///< Bit 15: even parity bit
constexpr uint16_t RW_MASK     = 0x4000u; ///< Bit 14 in command frame: 1=read, 0=write
constexpr uint16_t EF_MASK     = 0x4000u; ///< Bit 14 in response frame: error flag
constexpr uint16_t DATA_MASK   = 0x3FFFu; ///< Bits 13:0: angle count or register data

// ---------------------------------------------------------------------------
// Protocol helpers
// ---------------------------------------------------------------------------

/**
 * @brief Builds a read command word with correct even parity for a register address.
 *
 * @details Sets bit 14 (R/W=read), masks the address to 14 bits, then computes
 * the even parity bit so that `popcount(result) % 2 == 0`.
 *
 * @param reg_addr 14-bit register address (e.g., `REG_ANGLECOM`).
 * @returns 16-bit command word ready for transmission.
 *
 * @note The command used in the original codebase (`(0b11 << 14) | 0x3FFF` = 0xFFFF)
 *       is equivalent to `make_read_cmd(REG_ANGLECOM)` — all bits are 1, popcount=16
 *       (even), so parity bit=1. Both forms are correct.
 */
inline uint16_t make_read_cmd(uint16_t reg_addr)
{
    uint16_t word = static_cast<uint16_t>(RW_MASK | (reg_addr & DATA_MASK));
    uint16_t p    = static_cast<uint16_t>(__builtin_popcount(word) & 1u);
    return static_cast<uint16_t>((p << 15u) | word);
}

/**
 * @brief Verifies even parity of a 16-bit frame.
 *
 * @returns `true` if parity is correct (total popcount is even), `false` on error.
 */
inline bool parity_ok(uint16_t word)
{
    return (__builtin_popcount(word) & 1u) == 0u;
}

/**
 * @brief Returns `true` if the error flag (EF, bit 14) is set in a response frame.
 *
 * @details EF=1 means the sensor has a pending fault. Faults persist until
 * `REG_ERRFL` is read; the specific error type is encoded there.
 */
inline bool has_error_flag(uint16_t word)
{
    return (word & EF_MASK) != 0u;
}

/**
 * @brief Extracts the 14-bit angle / data field from a response frame.
 */
inline uint16_t data_field(uint16_t word)
{
    return word & DATA_MASK;
}

} // namespace ams5x47


// ===========================================================================
// iMXRT1062 LPSPI / DMAMUX helpers  (Teensy 4.x internal)
// ===========================================================================

namespace spi_enc_detail {

/**
 * @brief Maps a Teensyduino `SPIClass` reference to its iMXRT1062 LPSPI peripheral.
 *
 * | SPIClass | LPSPI  | Teensy 4.x pins               |
 * |----------|--------|-------------------------------|
 * | SPI      | LPSPI4 | MOSI=11, MISO=12, SCK=13      |
 * | SPI1     | LPSPI3 | MOSI=26, MISO=1,  SCK=27      |
 * | SPI2     | LPSPI1 | MOSI=43, MISO=42, SCK=45      |
 *
 * @returns Pointer to the LPSPI register struct, or `nullptr` if unrecognised.
 */
inline IMXRT_LPSPI_t* lpspi_for(SPIClass& spi)
{
    if (&spi == &SPI)  return &IMXRT_LPSPI4_S;
    if (&spi == &SPI1) return &IMXRT_LPSPI3_S;
    if (&spi == &SPI2) return &IMXRT_LPSPI1_S;
    return nullptr;
}

/**
 * @brief Returns the DMAMUX RX request-line index for a given SPIClass.
 *
 * @note Source indices are from iMXRT1062 RM Table 4-3. Verify
 *       `DMAMUX_SOURCE_LPSPIx_RX` constants against the installed `<imxrt.h>`.
 * @returns DMAMUX source index, or 255 if the SPIClass is unrecognised.
 */
inline uint8_t dmamux_rx_for(SPIClass& spi)
{
    if (&spi == &SPI)  return DMAMUX_SOURCE_LPSPI4_RX;
    if (&spi == &SPI1) return DMAMUX_SOURCE_LPSPI3_RX;
    if (&spi == &SPI2) return DMAMUX_SOURCE_LPSPI1_RX;
    return 255u;
}

} // namespace spi_enc_detail


// ===========================================================================
// SPIEncoder
// ===========================================================================

/**
 * @brief SPI absolute encoder driver for AMS AS5047P / AS5147P.
 *
 * See file-level documentation for the full protocol, validation pipeline, and
 * DMA async / prefetch usage.
 */
class SPIEncoder : public AbsoluteEncoder
{
public:

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    SPIEncoder() = default;
    ~SPIEncoder() = default;

    /**
     * @brief Constructs and initialises the SPI encoder.
     *
     * @param read_cmd   16-bit command for the angle register. Use
     *                   `ams5x47::make_read_cmd(ams5x47::REG_ANGLECOM)` to
     *                   build a correctly-parited command, or pass the legacy
     *                   constant `(0b11 << 14) | 0x3FFF` (= 0xFFFF) directly.
     * @param spi        SPI bus instance (`SPI`, `SPI1`, or `SPI2`).
     * @param chip_select Arduino pin used as CS (active-low). May be any GPIO.
     * @param speed      SPI clock [Hz]. Default: 10 MHz (within AS5047P spec at 3.3 V).
     * @param bit_order  `MSBFIRST` or `LSBFIRST`. Default: `MSBFIRST`.
     * @param mode       SPI mode. Default: `SPI_MODE1` (CPOL=0, CPHA=1).
     * @param res        Encoder resolution [bits]. Default: 14 (→ 16384 counts/rev).
     */
    SPIEncoder(
        uint16_t  read_cmd,
        SPIClass& spi,
        int       chip_select,
        int       speed     = 10'000'000,
        int       bit_order = MSBFIRST,
        int       mode      = SPI_MODE1,
        int       res       = 14
    )
    : read_cmd_  (read_cmd),
      spi_       (&spi),
      settings_  (SPISettings(speed, bit_order, mode)),
      cs_        (chip_select),
      max_count_ (1 << res)
    {
        pinMode(cs_, OUTPUT);
        digitalWriteFast(cs_, HIGH);
        spi_->begin();
        spi_->beginTransaction(settings_);
    }

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    /**
     * @brief Enables or disables per-frame even-parity verification (default: enabled).
     *
     * When enabled, any response frame with an odd total popcount is treated as
     * invalid (returns −1.0). Disable only for debugging or bench testing without
     * a connected sensor.
     */
    void set_parity_check_state(bool state)     { parity_check_enable_  = state; }

    /**
     * @brief Enables or disables checking the AS5047P error flag bit 14 (default: enabled).
     *
     * When enabled, any frame with EF=1 is treated as invalid (returns −1.0).
     * After an EF, read `ams5x47::REG_ERRFL` to identify the specific fault
     * (SPI framing error, offset loop error, CORDIC overflow, etc.).
     * Disable during start-up if the sensor clears a transient EF after the
     * first few reads.
     */
    void set_error_flag_check_state(bool state) { ef_check_enable_       = state; }

    /**
     * @brief Enables or disables the saturation-code glitch filter (default: enabled).
     *
     * When enabled, raw counts of 0 and (2^res − 1) are rejected. These are the
     * two saturation codes output by the AS5047P when the AGC reaches its limits
     * due to an out-of-range magnetic field. They do not represent valid positions.
     */
    void set_glitch_filter_state(bool state)    { glitch_filter_enable_  = state; }

    // -----------------------------------------------------------------------
    // DMA async initialisation
    // -----------------------------------------------------------------------

    /**
     * @brief Configures a DMA channel for asynchronous two-frame reads.
     *
     * @details Must be called **once** after construction and before any call to
     * `trigger_async_read()`. After this call, `read()` returns a cached DMA
     * result (if ready) before falling back to a blocking read.
     *
     * The supplied `DMAChannel` must have **persistent lifetime** (global or
     * static member). The encoder takes exclusive ownership of the channel.
     *
     * @param ch  Persistent `DMAChannel` object (allocated by the caller).
     * @returns `true` on success; `false` if `spi_` is not `SPI`, `SPI1`, or
     *          `SPI2` (LPSPI mapping unknown), or if the instance registry is full.
     *
     * @note DMA async requires iMXRT1062 (Teensy 4.x). The blocking `read()` and
     *       `read_raw()` paths remain fully functional without calling this.
     *
     * @warning Interrupt priority: the DMA completion ISR must have priority ≥ the
     *          control ISR that calls `trigger_async_read()` / `read_async()`, or
     *          the state machine will stall. Set the NVIC priority for the DMA
     *          channel accordingly via `NVIC_SET_PRIORITY`.
     */
    bool begin_dma(DMAChannel& ch)
    {
        lpspi_ = spi_enc_detail::lpspi_for(*spi_);
        if (!lpspi_) return false;

        dmamux_rx_ = spi_enc_detail::dmamux_rx_for(*spi_);
        dma_ch_    = &ch;
        dma_state_ = DMA_IDLE;

        if (num_instances_ >= MAX_INSTANCES) return false;
        instance_idx_                 = static_cast<int8_t>(num_instances_);
        instances_[num_instances_++]  = this;

        // Allocate the DMA channel once; trigger_async_read() will reconfigure it
        // per frame without re-allocating.
        ch.begin();
        ch.disable();

        return true;
    }

    // -----------------------------------------------------------------------
    // DMA async read path
    // -----------------------------------------------------------------------

    /**
     * @brief Initiates an asynchronous two-frame SPI read via DMA + interrupt chain.
     *
     * @details Executes the AS5047P two-frame protocol without blocking the caller:
     *
     * 1. Asserts CS, writes `read_cmd_` to LPSPI TDR, enables LPSPI RX DMA → DMA
     *    transfers the (discarded) frame-1 RX word and fires `handle_frame1_complete()`.
     * 2. `handle_frame1_complete()` (DMA ISR): toggles CS, waits 400 ns, reasserts
     *    CS, writes 0x0000 to TDR, reconfigures DMA to capture the angle word.
     * 3. Frame-2 DMA completes → `handle_frame2_complete()` (DMA ISR): deasserts CS,
     *    stores result in `dma_result_`, sets `dma_state_` to `DMA_RESULT_READY`.
     *
     * Call this at the **end** of control ISR N to prefetch for ISR N+1. Then call
     * `read_async()` at the start of ISR N+1 for a zero-wait angle read.
     *
     * @note No-op if `begin_dma()` has not been called, or if a previous async read
     *       is still in progress.
     */
    void trigger_async_read()
    {
        if (!lpspi_ || !dma_ch_)                                 return;
        if (dma_state_ == DMA_FRAME1_ACTIVE ||
            dma_state_ == DMA_FRAME2_ACTIVE)                     return;

        dma_state_ = DMA_FRAME1_ACTIVE;

        // Flush any stale data in the LPSPI RX FIFO before enabling DMA.
        while (lpspi_->SR & LPSPI_SR_RDF) { (void)lpspi_->RDR; }

        // Configure DMA for frame-1 RX: discard into dma_discard_.
        _configure_dma_for_frame(dma_discard_, frame1_isrs_[instance_idx_]);

        // Enable LPSPI RX DMA trigger.
        lpspi_->DER |= LPSPI_DER_RDDE;

        // Assert CS and start frame 1.
        lpspi_->TCR = (lpspi_->TCR & 0xFFFFF000u) | LPSPI_TCR_FRAMESZ(15u);
        digitalWriteFast(cs_, LOW);
        lpspi_->TDR = read_cmd_;
    }

    /**
     * @brief Returns the result of the last completed async read (non-blocking).
     *
     * @returns Validated angle [rad] in [0, 2π), or −1.0 if no result is ready
     *          or the frame failed validation (parity, EF, glitch filter).
     *
     * @note Clears the `DMA_RESULT_READY` state so the next `trigger_async_read()`
     *       may proceed. Call exactly once per ISR cycle after `trigger_async_read()`.
     */
    float read_async()
    {
        if (dma_state_ != DMA_RESULT_READY) return -1.f;
        dma_state_ = DMA_IDLE;
        return _convert(dma_result_);
    }

    /** @returns `true` if a DMA result is waiting to be consumed by `read_async()`. */
    bool dma_result_ready() const { return dma_state_ == DMA_RESULT_READY; }

    /** @returns `true` if an async read is currently in progress. */
    bool dma_busy() const
    {
        return dma_state_ == DMA_FRAME1_ACTIVE || dma_state_ == DMA_FRAME2_ACTIVE;
    }

    // -----------------------------------------------------------------------
    // Blocking read path  (AbsoluteEncoder interface)
    // -----------------------------------------------------------------------

    /**
     * @brief Reads the current shaft angle.
     *
     * @details If DMA is configured and a result is already ready, returns it
     * immediately (non-blocking). If an async read is in progress, spin-waits for
     * completion (< 4 µs). Otherwise falls back to the synchronous two-frame read.
     *
     * The returned angle passes through the full validation pipeline (parity, EF,
     * glitch filter). Invalid frames return −1.0.
     *
     * @returns Shaft angle [rad] in [0, 2π), or −1.0 on a filtered / invalid frame.
     */
    float read() override
    {
        if (dma_ch_) {
            // Spin-wait for any in-flight DMA to finish (avoids SPI bus conflict).
            while (dma_state_ == DMA_FRAME1_ACTIVE || dma_state_ == DMA_FRAME2_ACTIVE) { /* ~µs */ }
            if (dma_state_ == DMA_RESULT_READY) return read_async();
        }
        return _convert(read_raw());
    }

    /**
     * @brief Executes the full two-frame SPI protocol and returns the raw 16-bit
     *        response frame (blocking).
     *
     * @details The returned word includes the parity bit (15) and EF bit (14).
     * No validation is applied; the caller receives the raw sensor output.
     *
     * - Frame 1: CS↓, `transfer16(read_cmd_)`, CS↑, 400 ns gap.
     * - Frame 2: CS↓, `transfer16(0x0000)`, CS↑.
     *
     * Approximately 3–4 µs at 10 MHz SPI.
     *
     * @returns Raw 16-bit response frame from frame 2.
     */
    uint16_t read_raw()
    {
        digitalWriteFast(cs_, LOW);
        spi_->transfer16(read_cmd_);
        digitalWriteFast(cs_, HIGH);
        delayNanoseconds(400);
        digitalWriteFast(cs_, LOW);
        uint16_t raw = spi_->transfer16(0x0000u);
        digitalWriteFast(cs_, HIGH);
        return raw;
    }

private:

    // -----------------------------------------------------------------------
    // Validation and conversion
    // -----------------------------------------------------------------------

    /**
     * @brief Applies the full validation pipeline and converts a raw frame to radians.
     *
     * @details Validation order (any failure returns −1.0):
     *  1. Even-parity check  (if `parity_check_enable_`)
     *  2. Error-flag check   (if `ef_check_enable_`)
     *  3. Glitch filter      (if `glitch_filter_enable_`)
     *
     * @param raw  Raw 16-bit response frame from the sensor.
     * @returns Angle [rad] in [0, 2π), or −1.0 on any validation failure.
     */
    float _convert(uint16_t raw) const
    {
        if (parity_check_enable_ && !ams5x47::parity_ok(raw))      return -1.f;
        if (ef_check_enable_     && ams5x47::has_error_flag(raw))   return -1.f;

        uint16_t counts = ams5x47::data_field(raw);
        if (glitch_filter_enable_ &&
            (counts == 0u || counts == static_cast<uint16_t>(max_count_ - 1)))
            return -1.f;

        return static_cast<float>(counts)
               / static_cast<float>(max_count_ - 1)
               * _2_PI_;
    }

    // -----------------------------------------------------------------------
    // DMA helpers
    // -----------------------------------------------------------------------

    /** @brief (Re)configures the shared DMA channel for a single 16-bit RX transfer. */
    void _configure_dma_for_frame(volatile uint16_t& dest, void (*isr_fn)())
    {
        dma_ch_->disable();
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wstrict-aliasing"
        dma_ch_->source(*reinterpret_cast<volatile uint16_t*>(&lpspi_->RDR));
        #pragma GCC diagnostic pop
        dma_ch_->destination(dest);
        dma_ch_->transferSize(2);
        dma_ch_->transferCount(1);
        dma_ch_->triggerAtHardwareEvent(dmamux_rx_);
        dma_ch_->attachInterrupt(isr_fn);
        dma_ch_->enable();
    }

    // -----------------------------------------------------------------------
    // DMA ISR handlers  (placed in ITCM for minimal ISR latency)
    // -----------------------------------------------------------------------

    /**
     * @brief Frame-1 DMA completion: toggles CS, waits tCSn, starts frame 2.
     *
     * Runs at DMA interrupt priority. Must complete before the next SPI clock edge,
     * so it is placed in ITCM (`FASTRUN`).
     */
    FASTRUN void handle_frame1_complete()
    {
        dma_ch_->clearInterrupt();
        lpspi_->DER &= ~LPSPI_DER_RDDE;

        digitalWriteFast(cs_, HIGH);
        delayNanoseconds(400); // tCSn ≥ 400 ns (AS5047P datasheet §6.5)

        // Reconfigure DMA for frame 2: capture angle into dma_result_.
        dma_state_ = DMA_FRAME2_ACTIVE;
        _configure_dma_for_frame(dma_result_, frame2_isrs_[instance_idx_]);
        lpspi_->DER |= LPSPI_DER_RDDE;

        // Assert CS and start frame 2.
        lpspi_->TCR = (lpspi_->TCR & 0xFFFFF000u) | LPSPI_TCR_FRAMESZ(15u);
        digitalWriteFast(cs_, LOW);
        lpspi_->TDR = 0x0000u;
    }

    /**
     * @brief Frame-2 DMA completion: deasserts CS, marks result ready.
     *
     * After this returns, `dma_result_` holds the raw angle frame and
     * `dma_state_` is `DMA_RESULT_READY`. `read_async()` may now be called.
     */
    FASTRUN void handle_frame2_complete()
    {
        dma_ch_->clearInterrupt();
        lpspi_->DER &= ~LPSPI_DER_RDDE;
        digitalWriteFast(cs_, HIGH);
        dma_state_ = DMA_RESULT_READY;
    }

    // -----------------------------------------------------------------------
    // Static ISR trampoline dispatch
    // -----------------------------------------------------------------------

    // Up to MAX_INSTANCES encoders may use DMA concurrently.
    // Trampoline functions are required because DMAChannel::attachInterrupt()
    // accepts a raw function pointer (no captures/closures).
    //
    // NOTE: Static member definitions appear at the bottom of this file,
    // below the class. This is safe for the single-translation-unit Arduino
    // build model where all source is compiled as one unit.

    static constexpr uint8_t MAX_INSTANCES = 4;
    static SPIEncoder* instances_[MAX_INSTANCES];
    static uint8_t     num_instances_;

    static void f1_isr_0() { if (instances_[0]) instances_[0]->handle_frame1_complete(); }
    static void f1_isr_1() { if (instances_[1]) instances_[1]->handle_frame1_complete(); }
    static void f1_isr_2() { if (instances_[2]) instances_[2]->handle_frame1_complete(); }
    static void f1_isr_3() { if (instances_[3]) instances_[3]->handle_frame1_complete(); }

    static void f2_isr_0() { if (instances_[0]) instances_[0]->handle_frame2_complete(); }
    static void f2_isr_1() { if (instances_[1]) instances_[1]->handle_frame2_complete(); }
    static void f2_isr_2() { if (instances_[2]) instances_[2]->handle_frame2_complete(); }
    static void f2_isr_3() { if (instances_[3]) instances_[3]->handle_frame2_complete(); }

    using isr_fn = void (*)();

    static constexpr isr_fn frame1_isrs_[MAX_INSTANCES] = {
        f1_isr_0, f1_isr_1, f1_isr_2, f1_isr_3
    };
    static constexpr isr_fn frame2_isrs_[MAX_INSTANCES] = {
        f2_isr_0, f2_isr_1, f2_isr_2, f2_isr_3
    };

    // -----------------------------------------------------------------------
    // DMA state
    // -----------------------------------------------------------------------

    enum DmaState : uint8_t {
        DMA_IDLE,           ///< No async read in progress; result may be stale.
        DMA_FRAME1_ACTIVE,  ///< Frame 1 (command) clocking; DMA awaits RX.
        DMA_FRAME2_ACTIVE,  ///< Frame 2 (NOP / angle) clocking; DMA awaits RX.
        DMA_RESULT_READY,   ///< Frame 2 complete; dma_result_ holds raw angle.
    };

    // -----------------------------------------------------------------------
    // Members
    // -----------------------------------------------------------------------

    uint16_t           read_cmd_              = 0;
    SPIClass*          spi_                   = nullptr;
    SPISettings        settings_;
    int                cs_                    = 0;
    int                max_count_             = (1 << 14);

    bool               parity_check_enable_   = true;
    bool               ef_check_enable_       = true;
    bool               glitch_filter_enable_  = true;

    // DMA (null / 255 when DMA not configured)
    IMXRT_LPSPI_t*     lpspi_                 = nullptr;
    uint8_t            dmamux_rx_             = 255u;
    DMAChannel*        dma_ch_                = nullptr;
    volatile DmaState  dma_state_             = DMA_IDLE;
    volatile uint16_t  dma_discard_           = 0u; ///< Frame-1 RX goes here (discarded)
    volatile uint16_t  dma_result_            = 0u; ///< Frame-2 RX angle word
    int8_t             instance_idx_          = -1;
};


// ===========================================================================
// Static member definitions
// ===========================================================================
// These live in the header because this project uses the Arduino single-TU
// build model (one .cpp includes all headers). If this header is ever included
// from multiple translation units, move these definitions to spi_encoder.cpp.

SPIEncoder* SPIEncoder::instances_[SPIEncoder::MAX_INSTANCES]  = {};
uint8_t     SPIEncoder::num_instances_                         = 0;

// constexpr static arrays: out-of-line definition required in C++14 but not
// in C++17. The Teensy toolchain defaults to gnu++14; add explicit definitions
// if the linker complains about undefined references.
constexpr SPIEncoder::isr_fn SPIEncoder::frame1_isrs_[SPIEncoder::MAX_INSTANCES];
constexpr SPIEncoder::isr_fn SPIEncoder::frame2_isrs_[SPIEncoder::MAX_INSTANCES];


#endif // SPI_ENCODER_HPP
