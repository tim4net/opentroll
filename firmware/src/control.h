#pragma once
#include <stdint.h>

// ============== CONTROL ALGORITHMS ==============

// Throttle curve: maps raw ADC (0-4095) to PWM duty (0-255)
// Compensates for motor's nonlinear thrust response
// - Dead zone below 5% (motor can't start)
// - Exponential curve for finer low-speed control
// - Maps to 64-255 range (25% min PWM to actually spin the motor)
inline uint8_t throttle_curve(uint16_t pot_raw) {
    if (pot_raw < 205) return 0;  // 5% dead zone

    float pct = (float)(pot_raw - 205) / (float)(4095 - 205);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 1.0f) pct = 1.0f;

    pct = powf(pct, 1.5f);  // Exponential — finer control at low speeds
    // Map to 64-255 (motor needs minimum ~25% PWM to start spinning)
    return (uint8_t)(64.0f + pct * (255.0f - 64.0f));
}

// Soft start: ramp PWM toward target, never jump
// Rate 0.05/cycle at 20Hz = ~2 seconds to full (Round 2 fix R2-C7)
#define SOFT_START_RATE 0.05f

inline uint8_t soft_start(uint8_t target, uint8_t current) {
    if (target > current) {
        uint8_t step = (uint8_t)((float)(target - current) * SOFT_START_RATE);
        if (step < 1) step = 1;
        return current + step;
    }
    // Instant cut down (safety)
    return target;
}

// PID controller with anti-windup
struct PIDState {
    float integral;
    float prev_error;
    float kp, ki, kd;
    float integral_limit;
};

inline void pid_init(PIDState& pid, float kp, float ki, float kd, float int_limit) {
    pid.integral = 0;
    pid.prev_error = 0;
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.integral_limit = int_limit;
}

inline float pid_compute(PIDState& pid, float error, bool freeze_integral = false) {
    // Anti-windup: freeze integral when in dead band (Round 2 fix R2-I4)
    if (!freeze_integral) {
        pid.integral += error;
        if (pid.integral > pid.integral_limit) pid.integral = pid.integral_limit;
        if (pid.integral < -pid.integral_limit) pid.integral = -pid.integral_limit;
    }

    float derivative = error - pid.prev_error;
    pid.prev_error = error;

    float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    return output;
}

inline void pid_reset(PIDState& pid) {
    pid.integral = 0;
    pid.prev_error = 0;
}

// ============== BATTERY MONITORING (Round 3 safety review) ==============
// Divider 120kΩ/30kΩ (spec v3) → ratio 5.0
// At 14.6V (full LiFePO4): 2.92V on ADC — safe.
// (The old 100k/33k divider put 3.62V on GPIO34 at full charge — above the
//  ESP32 ADC's 3.3V range and at the pin's absolute maximum. Do not use it.)
// NOTE: spec v3 line "Simplified: voltage = adc_raw * 0.01612" is wrong —
// correct constant is 5.0 * 3.3 / 4095 = 0.004029.
#define BATT_DIVIDER_RATIO 5.0f

inline float adc_to_battery_voltage(uint16_t adc_raw) {
    return (float)adc_raw * BATT_DIVIDER_RATIO * 3.3f / 4095.0f;
}

// Moving average of 16 samples (spec v3) — single reads are noisy under motor load
#define BATT_FILTER_LEN 16
struct BattFilter {
    float samples[BATT_FILTER_LEN];
    uint8_t idx;
    uint8_t count;
};

inline void batt_filter_init(BattFilter& f) { f.idx = 0; f.count = 0; }

inline float batt_filter_update(BattFilter& f, float v) {
    f.samples[f.idx] = v;
    f.idx = (uint8_t)((f.idx + 1) % BATT_FILTER_LEN);
    if (f.count < BATT_FILTER_LEN) f.count++;
    float sum = 0;
    for (uint8_t i = 0; i < f.count; i++) sum += f.samples[i];
    return sum / (float)f.count;
}

// Battery state machine with hysteresis (spec CS-5):
// - Cutoff at 10.5V → CRITICAL (latched)
// - Recovery requires >11.5V sustained for 2s → drops to LOW (manual re-arm required by caller)
// - LOW below 11.5V → caller applies limp mode (50% throttle cap)
enum BattState { BATT_OK = 0, BATT_LOW = 1, BATT_CRITICAL = 2 };

struct BattMonitor {
    BattState state;
    uint32_t recovery_start_ms;
};

inline void batt_monitor_init(BattMonitor& m) { m.state = BATT_OK; m.recovery_start_ms = 0; }

inline BattState batt_monitor_update(BattMonitor& m, float voltage, uint32_t now_ms) {
    if (m.state == BATT_CRITICAL) {
        if (voltage > 11.5f) {
            if (m.recovery_start_ms == 0) m.recovery_start_ms = now_ms;
            else if (now_ms - m.recovery_start_ms >= 2000) {
                m.state = BATT_LOW;   // recovered — caller must go to DISARMED, not straight to drive
                m.recovery_start_ms = 0;
            }
        } else {
            m.recovery_start_ms = 0;  // dipped again — restart the 2s clock
        }
    } else {
        if (voltage < 10.5f) { m.state = BATT_CRITICAL; m.recovery_start_ms = 0; }
        else if (voltage < 11.5f) m.state = BATT_LOW;
        else m.state = BATT_OK;
    }
    return m.state;
}

// Limp mode (spec CS-5): low battery caps throttle at 50%
inline uint8_t apply_limp_mode(uint8_t pwm, BattState bs) {
    return (bs == BATT_LOW && pwm > 127) ? (uint8_t)127 : pwm;
}

// ============== HEADING SOURCE SELECTION (CS-3) ==============
// GPS COG is noise at low speed; compass is the truth when slow.
// COG only above 0.8 m/s, compass below 0.3 m/s, hysteresis between
// (keep previous source in the 0.3-0.8 band to prevent rapid switching).
enum HeadingSource { HDG_COMPASS = 0, HDG_COG = 1 };

inline HeadingSource select_heading_source(HeadingSource prev, float sog_ms) {
    if (sog_ms >= 0.8f) return HDG_COG;
    if (sog_ms <= 0.3f) return HDG_COMPASS;
    return prev;  // hysteresis band
}

// ============== ANCHOR BUTTON DEBOUNCE (CS-11) ==============
// Button state must be stable for 3 consecutive packets (~150ms at 20Hz)
// before the debounced value changes. Returns the debounced state.
struct AnchorDebounce {
    uint8_t stable_state;    // last debounced value
    uint8_t candidate;       // value being confirmed
    uint8_t count;           // consecutive packets at candidate
};

inline void anchor_debounce_init(AnchorDebounce& d) {
    d.stable_state = 0; d.candidate = 0; d.count = 0;
}

inline uint8_t anchor_debounce(AnchorDebounce& d, uint8_t raw) {
    if (raw == d.stable_state) {
        d.candidate = raw;
        d.count = 0;
        return d.stable_state;
    }
    if (raw == d.candidate) {
        if (++d.count >= 3) {
            d.stable_state = raw;
            d.count = 0;
        }
    } else {
        d.candidate = raw;
        d.count = 1;
    }
    return d.stable_state;
}

// seq_num wraparound-safe gap detection (Round 2 fix R2-I1)
inline uint16_t seq_gap(uint16_t received, uint16_t last) {
    uint16_t gap = (uint16_t)(received - last);
    return gap;
}

inline bool seq_is_newer(uint16_t received, uint16_t last) {
    uint16_t gap = seq_gap(received, last);
    return gap > 0 && gap < 32768;
}
