// =====================================================================
// stepper595.h — 3-channel stepper driver for MKS DLC32 v2.1
//
// The DLC32 v2.1 board does NOT wire ESP32 GPIOs directly to the
// stepper drivers. STEP/DIR/EN lines are driven through a 74HC595
// shift register over hardware SPI on these pins (matching the
// official MKS firmware and FluidNC config):
//
//   SCK   = GPIO16  (74HC595 SHCP / I2S BCK)
//   MOSI  = GPIO21  (74HC595 DS   / I2S DATA)
//   SS    = GPIO17  (74HC595 STCP / I2S WS / LATCH)
//
// 74HC595 bit layout (verified against MKS-DLC32-FIRMWARE
// `i2s_out_xyz_mks_dlc32.h` and FluidNC's I2SO bit numbering):
//
//   Q0 = global DISABLE (shared, ACTIVE HIGH → drivers off)
//   Q1 = X (ch1) STEP            Q2 = X (ch1) DIR
//   Q3 = Z (ch3) STEP            Q4 = Z (ch3) DIR
//   Q5 = Y (ch2) STEP            Q6 = Y (ch2) DIR
//   Q7 = unused
//
// Channel→axis mapping (so plug pump 1 in X, pump 2 in Y, pump 3 in Z):
//   ch index 0 → X driver socket
//   ch index 1 → Y driver socket
//   ch index 2 → Z driver socket
//
// One FreeRTOS task per channel runs the configured number of steps
// at the configured rate, then fires an optional completion callback.
// Every shift-register update (bit change + SPI write) is performed
// atomically under a mutex to prevent torn read-modify-write between
// concurrently running channels.
// =====================================================================
#pragma once

#include <Arduino.h>
#include <SPI.h>
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/semphr.h"
}

#ifndef STEPPER595_NUM_CH
#define STEPPER595_NUM_CH 3
#endif

class Stepper595 {
public:
    typedef void (*Callback)(int ch1based);

    inline void begin();
    inline void run(int ch, long steps, float stepsPerSec, bool cw, Callback onComplete = nullptr);
    inline void stop(int ch);
    inline void stopAll();

    inline bool isRunning(int ch) const {
        return ch >= 0 && ch < STEPPER595_NUM_CH && _ctx[ch].running;
    }
    inline bool anyRunning() const {
        for (int i = 0; i < STEPPER595_NUM_CH; i++) if (_ctx[i].running) return true;
        return false;
    }
    inline long stepsRemaining(int ch) const {
        return (ch >= 0 && ch < STEPPER595_NUM_CH) ? _ctx[ch].remaining : 0;
    }
    inline long stepsTotal(int ch) const {
        return (ch >= 0 && ch < STEPPER595_NUM_CH) ? _ctx[ch].total : 0;
    }

    inline void enableMotors();
    inline void disableMotors();

    // ---- Diagnostics ----
    // Pulse a single 74HC595 bit (Q1..Q7) at `hz` for `durationMs` ms,
    // with drivers enabled (Q0 LOW).  Used to map byte-bit → physical
    // driver socket.  Blocks the caller; do not run while a normal
    // motor task is in flight.
    inline void diagPulse(uint8_t bit, uint32_t hz, uint32_t durationMs);
    // Set a fixed bit (Q1..Q7) HIGH or LOW immediately.  Used to lock
    // a DIR line before pulsing the corresponding STEP line.
    inline void diagSetBit(uint8_t bit, bool high);

private:
    // ---- 74HC595 bit map (Qn → bit position in transmitted byte) ----
    static constexpr uint8_t BIT_DISABLE = 0;          // Q0 active HIGH
    // chIdx 0=X (pump1), 1=Y (pump2), 2=Z (pump3)
    static constexpr uint8_t stepBitFor(int ch) {
        // X=Q1, Y=Q5, Z=Q3 (matches MKS firmware I2SO bit assignments)
        return (ch == 0) ? 1 : (ch == 1) ? 5 : 3;
    }
    static constexpr uint8_t dirBitFor(int ch) {
        // X=Q2, Y=Q6, Z=Q4
        return (ch == 0) ? 2 : (ch == 1) ? 6 : 4;
    }
    // Q7 unused.

    // ---- SPI pin assignments (DLC32 v2.1) ----
    static constexpr int PIN_SCK   = 16;
    static constexpr int PIN_MOSI  = 21;
    static constexpr int PIN_LATCH = 17;

    static constexpr uint32_t SPI_HZ = 4000000;

    struct Ctx {
        TaskHandle_t      task          = nullptr;
        volatile bool     running       = false;
        volatile bool     abort         = false;
        volatile long     total         = 0;
        volatile long     remaining     = 0;
        volatile uint32_t halfPeriodUs  = 500;
        volatile bool     dirCw         = true;
        Callback          onComplete    = nullptr;
    };

    Ctx               _ctx[STEPPER595_NUM_CH];
    uint8_t           _bits      = 0;          // step/dir live bits (Q1..Q6 ; Q0/Q7 forced)
    bool              _enable    = false;      // logical enable (true=drivers on)
    SPIClass*         _spi       = nullptr;
    SemaphoreHandle_t _spiMutex  = nullptr;

    struct TaskArg { Stepper595* self; int ch; };
    TaskArg _taskArg[STEPPER595_NUM_CH];

    // Atomic: take mutex → modify shift-register state → SPI write → release.
    // setMask = bits to OR-in, clearMask = bits to AND-out (in our internal byte).
    // Q0 (DISABLE) and Q7 are *not* part of _bits; they are recomputed every push.
    //
    // The MKS DLC32 v2.1 wires *two* 74HC595 in daisy-chain (16 bits total):
    //   ESP32 MOSI → 595 #1 (steppers, Q0-Q7) → QH' → 595 #2 (beeper/LED/aux)
    // Sending only 8 bits per latch leaves chip #2's outputs holding stale data
    // shifted out of chip #1.  Those stale bits flicker on every transfer and
    // can phantom-drive any extra STEP/DIR lines fanned out from the aux chip,
    // which manifests as a *second* stepper moving when you only commanded one
    // (e.g. Y → Y+Z spinning in opposite directions).
    //
    // Fix: always shift 16 bits.  Upper byte = 0 (aux outputs disabled, beeper
    // off); lower byte = our stepper bits.  MSBFIRST + transfer16 puts the
    // lower byte at chip #1 after the latch — which is what we want.
    inline void writePort(uint8_t setMask, uint8_t clearMask) {
        if (xSemaphoreTake(_spiMutex, portMAX_DELAY) != pdTRUE) return;
        _bits = (_bits & ~clearMask) | setMask;
        uint8_t b = _bits & 0x7E;            // keep only Q1..Q6
        if (!_enable) b |= (1 << BIT_DISABLE); // Q0 HIGH ⇒ drivers disabled
        _spi->beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
        digitalWrite(PIN_LATCH, LOW);
        _spi->transfer16((uint16_t)b);       // 16-bit: upper=0 (aux 595), lower=b (stepper 595)
        digitalWrite(PIN_LATCH, HIGH);
        _spi->endTransaction();
        xSemaphoreGive(_spiMutex);
    }

    inline void pushState() { writePort(0, 0); }   // re-emit current bits (e.g. after enable toggle)

    inline void stepHigh(int ch)   { writePort(uint8_t(1 << stepBitFor(ch)), 0); }
    inline void stepLow (int ch)   { writePort(0, uint8_t(1 << stepBitFor(ch))); }
    inline void setDir  (int ch, bool cw) {
        // TMC2209 on the MKS DLC32 v2.1 reads DIR=LOW as CW, DIR=HIGH as CCW.
        // (Empirically verified — earlier code had this inverted, motors spun
        // backwards from the UI's CW button.)
        uint8_t m = 1 << dirBitFor(ch);
        if (cw) writePort(0, m);   // CW  → DIR LOW
        else    writePort(m, 0);   // CCW → DIR HIGH
    }

    static void taskTrampoline(void* arg) {
        TaskArg* a = (TaskArg*) arg;
        for (;;) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            a->self->doRun(a->ch);
        }
    }

    inline void doRun(int ch) {
        Ctx& c = _ctx[ch];
        c.running = true;
        c.abort   = false;

        // 1. Latch DIR + enable drivers (the very first call may need to flip Q0)
        setDir(ch, c.dirCw);
        if (!_enable) { _enable = true; pushState(); }
        // DIR setup time: stepper drivers want at most ~50 ns; 5 µs is plenty
        delayMicroseconds(5);

        const uint32_t halfUs = c.halfPeriodUs;
        const uint32_t pulseHigh = 5;                                  // µs STEP HIGH
        const uint32_t fullPeriodUs = halfUs * 2u;
        const uint32_t lowUs = (fullPeriodUs > pulseHigh) ? (fullPeriodUs - pulseHigh) : 1;

        while (c.remaining > 0 && !c.abort) {
            stepHigh(ch);
            delayMicroseconds(pulseHigh);
            stepLow(ch);

            // Sleep the rest of the period
            if (lowUs >= 2000) {
                // Long step interval — yield via vTaskDelay to free the core
                uint32_t ms = lowUs / 1000;
                vTaskDelay(pdMS_TO_TICKS(ms));
                uint32_t rem = lowUs - ms * 1000;
                if (rem) delayMicroseconds(rem);
            } else {
                delayMicroseconds(lowUs);
            }
            c.remaining--;
        }

        const bool aborted = c.abort;
        c.running = false;

        // If no other channel is still running, drop the global enable to save power/heat.
        bool any = false;
        for (int i = 0; i < STEPPER595_NUM_CH; i++) if (_ctx[i].running) { any = true; break; }
        if (!any) { _enable = false; pushState(); }

        if (!aborted && c.onComplete) c.onComplete(ch + 1);
    }
};

// ========================= public API ============================

inline void Stepper595::begin() {
    pinMode(PIN_LATCH, OUTPUT);
    digitalWrite(PIN_LATCH, HIGH);

    _spiMutex = xSemaphoreCreateMutex();

    // HSPI on remapped pins; MISO unused (-1), and SS=-1 too: passing GPIO17
    // as SS makes SPIClass call spiAttachSS() which enables HW CS — the SPI
    // peripheral then auto-toggles GPIO17 around every transfer in addition
    // to our manual digitalWrite, producing extra LATCH rising edges that
    // can capture half-shifted state into the 74HC595 outputs (this manifested
    // as Q5 toggling also triggering apparent activity on Q3 / Y → Z phantom
    // stepping).  Keep LATCH purely manual.
    _spi = new SPIClass(HSPI);
    _spi->begin(PIN_SCK, -1, PIN_MOSI, -1);

    _bits   = 0;
    _enable = false;
    pushState();   // pushes 0x01 → drivers disabled, all step/dir LOW

    // Spawn one task per channel
    for (int ch = 0; ch < STEPPER595_NUM_CH; ch++) {
        _taskArg[ch] = { this, ch };
        char name[16]; snprintf(name, sizeof(name), "stp595_%d", ch);
        xTaskCreatePinnedToCore(taskTrampoline, name, 3072, &_taskArg[ch], 5, &_ctx[ch].task, 1);
    }
}

inline void Stepper595::run(int ch, long steps, float stepsPerSec, bool cw, Callback onComplete) {
    if (ch < 0 || ch >= STEPPER595_NUM_CH) return;
    if (steps <= 0 || stepsPerSec <= 0.0f)  return;
    Ctx& c = _ctx[ch];

    // If a run is in flight on this channel, stop it first
    if (c.running) {
        c.abort = true;
        for (int i = 0; i < 50 && c.running; i++) vTaskDelay(pdMS_TO_TICKS(2));
    }

    c.total      = steps;
    c.remaining  = steps;
    c.dirCw      = cw;
    c.onComplete = onComplete;

    // Half-period in µs = 1e6 / (2 * stepsPerSec)
    float hp = 500000.0f / stepsPerSec;
    if (hp < 5.0f)        hp = 5.0f;          // upper bound on step rate
    if (hp > 500000.0f)   hp = 500000.0f;     // 1 Hz min
    c.halfPeriodUs = (uint32_t) hp;

    c.abort = false;
    xTaskNotifyGive(c.task);
}

inline void Stepper595::stop(int ch) {
    if (ch < 0 || ch >= STEPPER595_NUM_CH) return;
    _ctx[ch].abort = true;
}

inline void Stepper595::stopAll() {
    for (int i = 0; i < STEPPER595_NUM_CH; i++) _ctx[i].abort = true;
}

inline void Stepper595::enableMotors()  { _enable = true;  pushState(); }
inline void Stepper595::disableMotors() { _enable = false; pushState(); }

inline void Stepper595::diagPulse(uint8_t bit, uint32_t hz, uint32_t durationMs) {
    if (bit < 1 || bit > 7) return;
    if (hz == 0) hz = 100;
    if (hz > 2000) hz = 2000;
    if (durationMs > 5000) durationMs = 5000;

    bool prevEnable = _enable;
    _enable = true; pushState();
    delayMicroseconds(50);

    uint32_t halfUs = 500000UL / hz;
    if (halfUs < 5) halfUs = 5;
    uint8_t mask = (uint8_t)(1 << bit);

    uint32_t end = millis() + durationMs;
    while ((int32_t)(end - millis()) > 0) {
        writePort(mask, 0);
        delayMicroseconds(halfUs);
        writePort(0, mask);
        delayMicroseconds(halfUs);
    }

    _enable = prevEnable; pushState();
}

inline void Stepper595::diagSetBit(uint8_t bit, bool high) {
    if (bit < 1 || bit > 7) return;
    uint8_t m = (uint8_t)(1 << bit);
    if (high) writePort(m, 0);
    else      writePort(0, m);
}
