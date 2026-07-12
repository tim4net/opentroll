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

// seq_num wraparound-safe gap detection (Round 2 fix R2-I1)
inline uint16_t seq_gap(uint16_t received, uint16_t last) {
    uint16_t gap = (uint16_t)(received - last);
    return gap;
}

inline bool seq_is_newer(uint16_t received, uint16_t last) {
    uint16_t gap = seq_gap(received, last);
    return gap > 0 && gap < 32768;
}
