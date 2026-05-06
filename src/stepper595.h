// =====================================================================
// stepper595.h — 3-channel stepper driver for MKS DLC32 v2.1
//
// The DLC32 v2.1 board does NOT wire ESP32 GPIOs directly to the
// stepper drivers. STEP/DIR/EN lines are driven through a 74HC595
// shift register over hardware SPI:
//
//   SCK   = GPIO26  (74HC595 SHCP / BCK)
//   MOSI  = GPIO27  (74HC595 DS    / DATA)
//   SS    = GPIO25  (74HC595 STCP  / LATCH / WS)
//
// 74HC595 bit layout (matches FluidNC's MKS DLC32 v2.1 config):
//   Q0 = motor 0 STEP
//   Q1 = motor 0 DIR
//   Q2 = motor 1 STEP
//   Q3 = motor 1 DIR
//   Q4 = motor 2 STEP
//   Q5 = motor 2 DIR
//   Q6 = global ENABLE (active LOW; bit=0 enables drivers)
//   Q7 = unused
//
// One FreeRTOS task per channel. Each runs the configured number
// of steps at the configured rate, then fires an optional callback.
// All shift-register writes are guarded by a mutex.
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
    inline bool isRunning(int ch) const { return ch >= 0 && ch < STEPPER595_NUM_CH && _ctx[ch].running; }
    inline bool anyRunning() const {
        for (int i = 0; i < STEPPER595_NUM_CH; i++) if (_ctx[i].running) return true;
        return false;
    }
    inline long stepsRemaining(int ch) const { return (ch>=0&&ch<STEPPER595_NUM_CH)? _ctx[ch].remaining : 0; }
    inline long stepsTotal(int ch)     const { return (ch>=0&&ch<STEPPER595_NUM_CH)? _ctx[ch].total     : 0; }

    inline void enableMotors()  { _enable = true;  pushBits(); }
    inline void disableMotors() { _enable = false; pushBits(); }

private:
    struct Ctx {
        TaskHandle_t   task;
        volatile bool  running;
        volatile bool  abort;
        volatile long  total;
        volatile long  remaining;
        volatile uint32_t halfPeriodUs;  // half of step period
        volatile bool  dirCw;
        Callback       onComplete;
    };

    Ctx _ctx[STEPPER595_NUM_CH] = {};
    volatile uint8_t _bits = 0;       // current 74HC595 byte
    bool _enable = false;             // logical enable (true=drivers on)
    SPIClass* _spi = nullptr;
    SemaphoreHandle_t _spiMutex = nullptr;

    static constexpr int PIN_SCK   = 26;
    static constexpr int PIN_MOSI  = 27;
    static constexpr int PIN_LATCH = 25;

    inline void pushBits() {
        // Compose byte. Q6 = enable (active LOW => bit=0 when enabled).
        uint8_t b = _bits & 0x3F;
        if (!_enable) b |= (1 << 6);   // disable: drive EN high
        // Q7 unused, leave 0.
        if (xSemaphoreTake(_spiMutex, portMAX_DELAY) == pdTRUE) {
            _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
            digitalWrite(PIN_LATCH, LOW);
            _spi->transfer(b);
            digitalWrite(PIN_LATCH, HIGH);
            _spi->endTransaction();
            xSemaphoreGive(_spiMutex);
        }
    }

    inline void setStepBit(int ch, bool high) {
        uint8_t mask = 1 << (ch * 2);
        if (high) _bits |=  mask;
        else      _bits &= ~mask;
    }
    inline void setDirBit(int ch, bool cw) {
        uint8_t mask = 1 << (ch * 2 + 1);
        if (cw) _bits |=  mask;
        else    _bits &= ~mask;
    }

    static void taskTrampoline(void* arg) {
        struct Pack { Stepper595* self; int ch; };
        Pack* p = (Pack*) arg;
        Stepper595* self = p->self;
        int ch = p->ch;
        // free pack — note: we never delete because we never stop the task
        for (;;) {
            // Wait for a notification = a new run request
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            self->doRun(ch);
        }
    }

    inline void doRun(int ch) {
        Ctx& c = _ctx[ch];
        c.running = true;
        c.abort   = false;

        // Set DIR before running
        setDirBit(ch, c.dirCw);
        pushBits();
        delayMicroseconds(5);

        // Enable drivers if not already
        if (!_enable) { _enable = true; pushBits(); delayMicroseconds(20); }

        const uint32_t halfUs = c.halfPeriodUs;

        while (c.remaining > 0 && !c.abort) {
            // STEP HIGH ~5us
            setStepBit(ch, true);
            pushBits();
            delayMicroseconds(5);
            setStepBit(ch, false);
            pushBits();

            // Remaining low time = (2*halfUs) - 5us pulse - latency
            uint32_t lowUs = (halfUs * 2) > 5 ? (halfUs * 2 - 5) : 1;
            if (lowUs >= 1500) {
                // Long step interval: yield to other tasks
                vTaskDelay(pdMS_TO_TICKS(lowUs / 1000));
                uint32_t rem = lowUs % 1000;
                if (rem) delayMicroseconds(rem);
            } else {
                delayMicroseconds(lowUs);
            }
            c.remaining--;
        }

        bool aborted = c.abort;
        c.running = false;

        // If no other channel is running, drop enable to save power
        bool any = false;
        for (int i = 0; i < STEPPER595_NUM_CH; i++) if (_ctx[i].running) { any = true; break; }
        if (!any) { _enable = false; pushBits(); }

        if (!aborted && c.onComplete) c.onComplete(ch + 1);
    }
};

inline void Stepper595::begin() {
    pinMode(PIN_LATCH, OUTPUT);
    digitalWrite(PIN_LATCH, HIGH);

    _spiMutex = xSemaphoreCreateMutex();

    // Use HSPI bus, remapped to DLC32 pins (SCK=26, MOSI=27, SS=25, MISO=-1)
    _spi = new SPIClass(HSPI);
    _spi->begin(PIN_SCK, -1, PIN_MOSI, PIN_LATCH);

    // Initial state: enable high (drivers off), step/dir all 0
    _bits = 0;
    _enable = false;
    pushBits();

    // Spawn one task per channel
    static struct { Stepper595* self; int ch; } packs[STEPPER595_NUM_CH];
    for (int ch = 0; ch < STEPPER595_NUM_CH; ch++) {
        _ctx[ch].running = false;
        _ctx[ch].abort   = false;
        _ctx[ch].total = _ctx[ch].remaining = 0;
        _ctx[ch].onComplete = nullptr;
        packs[ch].self = this;
        packs[ch].ch   = ch;
        char name[16]; snprintf(name, sizeof(name), "stp595_%d", ch);
        xTaskCreatePinnedToCore(taskTrampoline, name, 3072, &packs[ch], 5, &_ctx[ch].task, 1);
    }
}

inline void Stepper595::run(int ch, long steps, float stepsPerSec, bool cw, Callback onComplete) {
    if (ch < 0 || ch >= STEPPER595_NUM_CH) return;
    if (steps <= 0 || stepsPerSec <= 0.0f) return;
    Ctx& c = _ctx[ch];
    if (c.running) {
        c.abort = true;
        // Wait briefly for prior run to acknowledge
        for (int i = 0; i < 50 && c.running; i++) vTaskDelay(pdMS_TO_TICKS(2));
    }
    c.total       = steps;
    c.remaining   = steps;
    c.dirCw       = cw;
    c.onComplete  = onComplete;
    // half period in microseconds: 1e6 / (2 * sps)
    float hp = 500000.0f / stepsPerSec;
    if (hp < 5.0f) hp = 5.0f;            // clamp to safe min
    if (hp > 500000.0f) hp = 500000.0f;  // 1Hz min
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
