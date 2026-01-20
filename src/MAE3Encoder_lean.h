#pragma once
#ifndef MAE3_ENCODER2_H
    #define MAE3_ENCODER2_H
    #include <Arduino.h>

    #include <atomic>
    #include <cstdint>

extern "C" {
    #include "driver/gpio.h" // ESP-IDF GPIO ISR service
    #include "esp_intr_alloc.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/portmacro.h"
    #include "soc/gpio_reg.h"
}

/// \file MAE3Encoder_lean.hpp
/// \brief MAE3 PWM encoder capture (header-only, ISR-light, no heap in RT
/// paths)
///
/// Contracts & Assumptions
/// - Initialization allocates/configures resources; no dynamic allocation afterwards.
/// - ISR path is minimal: only timestamp deltas and volatile stores (no math/printf).
/// - One active encoder channel at a time (per original design).
/// - Observer callbacks must be short and non-blocking; they run from a task context.
/// - For thread-safety: task-side critical sections use FreeRTOS portMUX spinlocks.
/// - If cycle counter timing is enabled, CPU frequency is sampled at init (best-effort).
///
/// Compliance highlights
/// - MISRA C++-friendly style, RAII where applicable, no blocking in ISRs.
/// - `enum class` instead of macros for logical constants; configuration via `constexpr`.
/// - Avoids `noInterrupts()/interrupts()`; uses FreeRTOS critical sections.
/// - No dynamic allocation at runtime (fixed-size arrays, atomics).

namespace mae3 {
// ------------------------------ Fast GPIO read ---------------------------------
inline bool fastRead(int pin) noexcept {
    if (pin < 32) {
        return (REG_READ(GPIO_IN_REG) >> pin) & 0x1U;
    }
    return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 0x1U;
}

// ------------------------------ Time base (ticks) ------------------------------
struct TimeStamp final {
    // Init must be called once during system setup (EncoderManager::configure does this).
    static inline void init() noexcept {
        const uint32_t mhz = getCpuFrequencyMhz();
        cycles_per_us_.store((mhz == 0U) ? 240U : mhz, std::memory_order_relaxed);
    }

    static inline uint32_t nowTicks() noexcept {
        return ESP.getCycleCount(); // 32-bit wrap is fine for deltas
    }

    static inline double_t toMicros(uint32_t v) noexcept {
        const uint32_t c = cycles_per_us_.load(std::memory_order_relaxed);
        const uint32_t cyclesPerUs = c == 0U ? 240U : c;                      // If cycles_per_us_ has not been initialized yet, fall back to 240 (ESP32 @ 240 MHz)
        return static_cast<double_t>(v) / static_cast<double_t>(cyclesPerUs); // Convert CPU cycles (ticks) into microseconds with fractional precision
    }

private:
    static inline std::atomic<uint32_t> cycles_per_us_{240U};
};

// ----------------------------- Types & constants -------------------------------
enum class Status : uint8_t { Ok = 0U, InvalidArg, NotInitialized, AlreadyInitialized };

enum class LogicLevel : uint8_t { Low = 0U, High = 1U };

struct Constants_MAE3 final {
    static constexpr uint32_t kResolutionBits = 12U;
    static constexpr uint32_t kResolutionSteps = (1UL << kResolutionBits);                          // 4096
    static constexpr double_t kMaxResolutionSteps_d = static_cast<double_t>(kResolutionSteps + 2U); // 4098.0f
    static constexpr double_t kMaxIndex_d = static_cast<double_t>(kResolutionSteps - 1U);           // 4095.0f
    static constexpr double_t kBorderIndex_d = static_cast<double_t>(kResolutionSteps - 2U);        // 4094.0f

    // Valid PWM frame bounds for MAE3 (typical ~4.0ms period). Keep generous margins.
    static constexpr double_t kMinTonUs_d = 0.95f;
    static constexpr double_t kMaxTonUs_d = 1.05f;
    static constexpr double_t kMinCycleUs_d = 3892.0f; // ~3.9 ms
    static constexpr double_t kMaxCycleUs_d = 4302.0f; // ~4.3 ms
    static constexpr double_t kZeroZoneUs_d = 7900.0f;
};

struct Constants_MAE4 final {
    static constexpr uint32_t kResolutionBits = 12U;
    static constexpr uint32_t kResolutionSteps = (1UL << kResolutionBits);                          // 4096
    static constexpr double_t kMaxResolutionSteps_d = static_cast<double_t>(kResolutionSteps + 2U); // 4098.0f
    static constexpr double_t kMaxIndex_d = static_cast<double_t>(kResolutionSteps - 1U);           // 4095.0f
    static constexpr double_t kBorderIndex_d = static_cast<double_t>(kResolutionSteps - 2U);        // 4094.0f

    // Valid PWM frame bounds for MAE3 (typical ~4.0ms period). Keep generous margins.
    static constexpr double_t kMinTonUs_d = 0.95f;
    static constexpr double_t kMaxTonUs_d = 1.05f;
    static constexpr double_t kMinCycleUs_d = 974.0f;
    static constexpr double_t kMaxCycleUs_d = 1076.0f;
    static constexpr double_t kZeroZoneUs_d = 7900.0f;
};

struct GpioConfig final {
    int32_t pin;
    bool pullup;
    bool pulldown; // Not relied upon by core logic; prefer external pulldown if needed
    bool inverted;
};

enum class EncoderType : uint8_t { MAE3 = 0, MAE4 = 1 };

class IEncoderObserver {
public:
    virtual ~IEncoderObserver() = default;
    /// \brief Called when a new position sample is available.
    /// \param index Logical encoder index
    /// \param position 0..4095 (12-bit)
    /// \param tonUs_f High-time (µs)
    /// \param toffUs_f Low-time (µs)
    virtual void onPositionUpdate(uint8_t index, double_t position_d, double_t tonUs_d, double_t toffUs_d) = 0;
};

// -------------------------- EdgeCapture (per channel) --------------------------
class EdgeCapture {
public:
    EdgeCapture() = default;

    // Task-side: reset capture state
    void reset(uint32_t now_ticks) noexcept {
        portENTER_CRITICAL(&s_mux_);
        last_edge_ticks_ = now_ticks;
        ton_ticks_ = 0U;
        toff_ticks_ = 0U;
        high_valid_ = false;
        low_valid_ = false;
        ready_ = false;
        portEXIT_CRITICAL(&s_mux_);
    }

    // ISR-side: store delta only (no math/log/heap)
    inline void onEdgeISR(bool level, uint32_t now_ticks) noexcept {
        const uint32_t dt = now_ticks - last_edge_ticks_;
        last_edge_ticks_ = now_ticks;
        if (level) {
            toff_ticks_ = dt;
            low_valid_ = true;
        } else {
            ton_ticks_ = dt;
            high_valid_ = true;
        }
        if (high_valid_ && low_valid_) {
            ready_ = true;
            high_valid_ = false;
            low_valid_ = false;
        }
    }

    // Task-side: consume most recent full sample (non-blocking)
    bool tryConsume(uint32_t &ton_ticks, uint32_t &toff_ticks) noexcept {
        portENTER_CRITICAL(&s_mux_);
        const bool rdy = ready_;
        if (rdy) {
            ton_ticks = ton_ticks_;
            toff_ticks = toff_ticks_;
            ready_ = false;
        }
        portEXIT_CRITICAL(&s_mux_);
        return rdy;
    }

private:
    // NOTE: These fields are touched from ISR; keep them trivially copyable and
    // small.
    volatile uint32_t last_edge_ticks_{0U};
    volatile uint32_t ton_ticks_{0U};
    volatile uint32_t toff_ticks_{0U};
    volatile bool high_valid_{false};
    volatile bool low_valid_{false};
    volatile bool ready_{false};

    // Task-side critical section (NOT used in ISR)
    static inline portMUX_TYPE s_mux_ = portMUX_INITIALIZER_UNLOCKED;
};

// ----------------------------- Mae3Encoder (per channel) -----------------------------
class Mae3Encoder {
public:
    Mae3Encoder() = default;
    Mae3Encoder(uint8_t index, const GpioConfig &cfg, EncoderType encType) : index_{index}, cfg_{cfg}, _inAbsenceOfInterruptFlag(false), encType_(encType) {}

    Status init() {
        if (inited_) {
            return Status::AlreadyInitialized;
        }
        pinMode(cfg_.pin, cfg_.pullup ? INPUT_PULLUP : INPUT);
        TimeStamp::init(); // Ensure time base initialized
        capture_.reset(TimeStamp::nowTicks());
        // Attach shared ISR using "this" as argument
        attachInterruptArg(digitalPinToInterrupt(cfg_.pin), &Mae3Encoder::isrShared, this, CHANGE);
        disable(); // keep disabled until selected active
        inited_ = true;
        return Status::Ok;
    }

    Status deinit() {
        if (!inited_) {
            return Status::NotInitialized;
        }
        detachInterrupt(digitalPinToInterrupt(cfg_.pin));
        inited_ = false;
        enabled_ = false;
        return Status::Ok;
    }

    Status enable() {
        if (!inited_) {
            return Status::NotInitialized;
        }
        enabled_ = true;
        return Status::Ok;
    }

    Status disable() {
        if (!inited_) {
            return Status::NotInitialized;
        }
        enabled_ = false;
        return Status::Ok;
    }

    double_t getResolutionSteps() {

        if (encType_ == EncoderType::MAE3)
            return static_cast<double_t>(Constants_MAE3::kResolutionSteps);

        else if (encType_ == EncoderType::MAE4)
            return static_cast<double_t>(Constants_MAE4::kResolutionSteps);

        return 0;
    }

    /// \brief Fetch last complete sample; returns false if nothing new.
    bool tryGetPosition(double_t &pos_out_d, double_t &tonUs_d, double_t &toffUs_d) noexcept {
        uint32_t ton_raw{0U}, toff_raw{0U};
        if (!capture_.tryConsume(ton_raw, toff_raw)) {
            return false;
        }

        double_t ton_us_d = TimeStamp::toMicros(ton_raw);
        double_t toff_us_d = TimeStamp::toMicros(toff_raw);
        _inAbsenceOfInterruptFlag = true;

        if (cfg_.inverted) {
            const double_t tmp_d = ton_us_d;
            ton_us_d = toff_us_d;
            toff_us_d = tmp_d;
        }

        pos_out_d = dutyToPosition(ton_us_d, toff_us_d, encType_);
        tonUs_d = ton_us_d;
        toffUs_d = toff_us_d;
        return true;
    }

    inline uint8_t index() const noexcept { return index_; }

    void attachInAbsenceOfInterrupt(void (*callback)()) { _inAbsenceOfInterrupt = callback; }
    void handleInAbsenceOfInterrupt() {
        if (_inAbsenceOfInterruptFlag) {
            _inAbsenceOfInterruptFlag = false;
            if (_inAbsenceOfInterrupt)
                _inAbsenceOfInterrupt();
        }
    }
    void setInAbsenceOfInterruptFlag(bool flag) { _inAbsenceOfInterruptFlag = flag; }

private:
    static void IRAM_ATTR isrShared(void *arg) {
        auto *self = static_cast<Mae3Encoder *>(arg);
        if ((self != nullptr) && self->enabled_) {
            // We trust hardware to supply CHANGE edges; read current level and
            // timestamp
            const bool level = (fastRead(self->cfg_.pin) != 0);
            self->capture_.onEdgeISR(level, TimeStamp::nowTicks());
        }
    }
    #if 0
    // Convert PWM high/low durations (µs) to 12-bit position (0..4095)
    static double_t dutyToPosition(double_t ton_us_d, double_t toff_us_d, EncoderType encTyp) noexcept { 
        const double_t tcycle_d = ton_us_d + toff_us_d;

        if (encTyp == EncoderType::MAE3) {
            if (tcycle_d > Constants_MAE3::kZeroZoneUs_d || ton_us_d < Constants_MAE3::kMinTonUs_d) {
                // Zero-range -> return 0 value
                last_valid_pos_d_.store(0.0f, std::memory_order_relaxed);
                return 0.0f;
            }
        #if 0
            if ((tcycle_d < Constants::kMinCycleUs_d) || (tcycle_d > Constants::kMaxCycleUs_d)) {
                // Out-of-range period -> return last valid value
                return last_valid_pos_d_.load(std::memory_order_relaxed);
            }
        #endif
        } else if (encTyp == EncoderType::MAE4) {
            if (tcycle_d > Constants_MAE4::kZeroZoneUs_d || ton_us_d < Constants_MAE4::kMinTonUs_d) {
                // Zero-range -> return 0 value
                last_valid_pos_d_.store(0.0f, std::memory_order_relaxed);
                return 0.0f;
            }
        #if 0
            if ((tcycle_d < Constants_MAE4::kMinCycleUs_d) || (tcycle_d > Constants_MAE4::kMaxCycleUs_d)) {
                // Out-of-range period -> return last valid value
                return last_valid_pos_d_.load(std::memory_order_relaxed);
            }
        #endif
        }

        double_t pos_d;

        if (encTyp == EncoderType::MAE3) {

            const double_t num_d = ton_us_d * Constants_MAE3::kMaxResolutionSteps_d; // Numbers in this range are safe in 32 bits

            double_t x_d = (num_d / tcycle_d); // floor
            x_d = (x_d > 0.0f) ? (x_d - 1.0f) : 0.0f;

            if (x_d <= Constants_MAE3::kBorderIndex_d)
                pos_d = x_d;
            else                                     // x >= 4095.0f
                pos_d = Constants_MAE3::kMaxIndex_d; // End clip according to datasheet

            last_valid_pos_d_.store(pos_d, std::memory_order_relaxed);

        } else if (encTyp == EncoderType::MAE4) {

            const double_t num_d = ton_us_d * Constants_MAE4::kMaxResolutionSteps_d; // Numbers in this range are safe in 32 bits

            double_t x_d = (num_d / tcycle_d); // floor
            x_d = (x_d > 0.0f) ? (x_d - 1.0f) : 0.0f;

            if (x_d <= Constants_MAE4::kBorderIndex_d)
                pos_d = x_d;
            else                                     // x >= 1023.0f
                pos_d = Constants_MAE4::kMaxIndex_d; // End clip according to datasheet

            last_valid_pos_d_.store(pos_d, std::memory_order_relaxed);
        }
        return pos_d;
    }
    #endif

    static inline double_t dutyToPosition(double_t ton_us_d, double_t toff_us_d, EncoderType encTyp) noexcept {
        const double_t tcycle_d = ton_us_d + toff_us_d;

        if (encTyp == EncoderType::MAE3) {
            if (tcycle_d > Constants_MAE3::kZeroZoneUs_d || ton_us_d < Constants_MAE3::kMinTonUs_d) {
                // Zero-range -> return 0 value
                last_valid_pos_d_.store(0.0f, std::memory_order_relaxed);
                return 0.0f;
            }
        } else if (encTyp == EncoderType::MAE4) {
            if (tcycle_d > Constants_MAE4::kZeroZoneUs_d || ton_us_d < Constants_MAE4::kMinTonUs_d) {
                // Zero-range -> return 0 value
                last_valid_pos_d_.store(0.0f, std::memory_order_relaxed);
                return 0.0f;
            }
        }

        double_t pos_d = 0.0;

        if (encTyp == EncoderType::MAE3) {
            const double_t num_d = ton_us_d * Constants_MAE3::kMaxResolutionSteps_d;

            double_t x_d = (num_d / tcycle_d);
            x_d = (x_d > 0.0) ? (x_d - 1.0) : 0.0;

            if (x_d <= Constants_MAE3::kBorderIndex_d)
                pos_d = x_d;
            else
                pos_d = Constants_MAE3::kMaxIndex_d;

            last_valid_pos_d_.store(pos_d, std::memory_order_relaxed);

        } else if (encTyp == EncoderType::MAE4) {

            // MAE4 datasheet-based duty decoding:
            // PWM cycle time is 4351 internal clock periods.
            // There are 4096 possible duty values:
            // duty_min = 128/4351, duty_max = 4223/4351.
            // Therefore: duty = (128 + code)/4351  =>  code = duty*4351 - 128.
            constexpr double_t kPwmCycleClocks_d = 4351.0;
            constexpr double_t kMinHighClocks_d = 128.0;
            constexpr double_t kMaxCode_d = 4095.0;

            if (tcycle_d <= 0.0) {
                // Invalid period -> return last valid value
                return last_valid_pos_d_.load(std::memory_order_relaxed);
            }

            const double_t duty_d = ton_us_d / tcycle_d; // 0..1
            double_t code_d = (duty_d * kPwmCycleClocks_d) - kMinHighClocks_d;

            // Clamp to [0..4095]
            if (code_d < 0.0)
                code_d = 0.0;
            if (code_d > kMaxCode_d)
                code_d = kMaxCode_d;

            // Optional: round to nearest integer code (keep as double if you want fractional)
            // code_d = floor(code_d + 0.5);

            pos_d = code_d;
            last_valid_pos_d_.store(pos_d, std::memory_order_relaxed);
        }

        return pos_d;
    }

    static inline std::atomic<double_t> last_valid_pos_d_{0.0f};

    uint8_t index_{0U};
    GpioConfig cfg_{};
    EncoderType encType_{};
    bool inited_{false};
    volatile bool enabled_{false};
    EdgeCapture capture_{};

    // Optional movement complete callback
    volatile bool _inAbsenceOfInterruptFlag;
    void (*_inAbsenceOfInterrupt)();
};

// ---------------- Manager (templated N; fixed capacity; one-active-at-a-time) ----------------
template <size_t N> class EncoderManager {
public:
    Status configure(const GpioConfig (&pins)[N]) {
        if (inited_) {
            return Status::AlreadyInitialized;
        }
        TimeStamp::init(); // Initialize time base once
        for (size_t i = 0U; i < N; ++i) {
            encs_[i] = Mae3Encoder(static_cast<uint8_t>(i),
                                   pins[i]); // RAII: stack object
            (void)encs_[i].init();
        }
        inited_ = true;
        return Status::Ok;
    }

    Mae3Encoder *get(size_t i) { return (i < N) ? &encs_[i] : nullptr; }

    // Only one encoder active at a time (as per module contract)
    Status setActive(size_t i) {
        if (!inited_ || (i >= N)) {
            return Status::InvalidArg;
        }
        for (auto &e : encs_) {
            (void)e.disable();
        }
        active_ = &encs_[i];
        return active_->enable();
    }

    // Observer management (thread-safe, fixed capacity)
    void attach(IEncoderObserver *obs) {
        portENTER_CRITICAL(&s_mux_);
        for (auto &s : observers_) {
            if (s == nullptr) {
                s = obs;
                break;
            }
        }
        portEXIT_CRITICAL(&s_mux_);
    }

    // Poll active encoder and notify observers (non-blocking)
    void pollAndNotify() {
        if (active_ == nullptr) {
            return;
        }

        double_t pos_d{0U};
        double_t ton_d{0U};
        double_t toff_d{0U};

        if (active_->tryGetPosition(pos_d, ton_d, toff_d)) {
            // Snapshot observer pointers under lock, then notify without holding the lock.
            IEncoderObserver *local[N]{};
            portENTER_CRITICAL(&s_mux_);
            for (size_t i = 0; i < N; ++i) {
                local[i] = observers_[i];
            }
            portEXIT_CRITICAL(&s_mux_);

            for (auto *o : local) {
                if (o != nullptr) {
                    o->onPositionUpdate(active_->index(), pos_d, ton_d, toff_d);
                }
            }
        }
    }

private:
    bool inited_{false};
    Mae3Encoder encs_[N];
    Mae3Encoder *active_{nullptr};
    IEncoderObserver *observers_[N]{}; // fixed-capacity, no dynamic allocation

    static inline portMUX_TYPE s_mux_ = portMUX_INITIALIZER_UNLOCKED;
};

} // namespace mae3
#endif