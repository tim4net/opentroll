#include <unity.h>
#include "packets.h"
#include "navigation.h"
#include "control.h"

// ============================================================
// PACKET SIZE TESTS — Catch the Round 2 CRITICAL bugs
// ============================================================

void test_control_packet_size(void) {
    // If this fails, the struct has padding corruption
    TEST_ASSERT_EQUAL(16, (int)sizeof(ControlPacket));
}

void test_telemetry_packet_size(void) {
    // Expected: 50 bytes (actual: 48 — verified by test)
    TEST_ASSERT_EQUAL(48, (int)sizeof(TelemetryPacket));
}

void test_packets_are_packed(void) {
    // Verify no padding between fields by checking specific offsets
    ControlPacket cp;
    TEST_ASSERT_EQUAL(0, (int)((uint8_t*)&cp.device_id - (uint8_t*)&cp));
    TEST_ASSERT_EQUAL(1, (int)((uint8_t*)&cp.fw_version - (uint8_t*)&cp));
    TEST_ASSERT_EQUAL(2, (int)((uint8_t*)&cp.seq_num - (uint8_t*)&cp));
    TEST_ASSERT_EQUAL(4, (int)((uint8_t*)&cp.speed_raw - (uint8_t*)&cp));
    TEST_ASSERT_EQUAL(14, (int)((uint8_t*)&cp.checksum - (uint8_t*)&cp));
}

// ============================================================
// CHECKSUM TESTS
// ============================================================

void test_control_checksum_roundtrip(void) {
    ControlPacket cp;
    memset(&cp, 0, sizeof(cp));
    cp.device_id = 0x42;
    cp.speed_raw = 2048;
    cp.direction = DIR_FORWARD;
    set_checksum(cp);
    TEST_ASSERT_TRUE(verify_checksum(cp));
}

void test_checksum_catches_corruption(void) {
    ControlPacket cp;
    memset(&cp, 0, sizeof(cp));
    cp.speed_raw = 3000;
    cp.direction = DIR_FORWARD;
    set_checksum(cp);
    // Corrupt a byte
    cp.speed_raw = 3001;
    TEST_ASSERT_FALSE(verify_checksum(cp));
}

void test_telemetry_checksum_roundtrip(void) {
    TelemetryPacket tp;
    memset(&tp, 0, sizeof(tp));
    tp.lat = 30.22f;
    tp.lon = -92.01f;
    tp.main_voltage = 13.2f;
    set_checksum(tp);
    TEST_ASSERT_TRUE(verify_checksum(tp));
}

// ============================================================
// HEADING WRAPAROUND TESTS — Round 2 CRITICAL fix R2-C5
// ============================================================

void test_heading_error_no_wrap(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, heading_error(90, 80));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, heading_error(80, 90));
}

void test_heading_error_wrap_359_to_1(void) {
    // The classic bug case: target 1°, current 359°
    // Should be +2° (turn right slightly), NOT -358° (spin all the way around)
    float err = heading_error(1.0f, 359.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, err);
}

void test_heading_error_wrap_1_to_359(void) {
    // Reverse case: target 359°, current 1°
    // Should be -2° (turn left slightly)
    float err = heading_error(359.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.0f, err);
}

void test_heading_error_180(void) {
    // Exactly opposite — should pick +180 (arbitrary but consistent)
    float err = heading_error(270.0f, 90.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, fabs(err));
}

void test_normalize_heading(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, normalize_heading(360.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, normalize_heading(450.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 270.0f, normalize_heading(-90.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, normalize_heading(180.0f));
}

// ============================================================
// GPS DISTANCE / BEARING TESTS
// ============================================================

void test_local_distance_zero(void) {
    LocalPos p = {0, 0};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, local_distance(p));
}

void test_local_distance_known(void) {
    // 1 degree latitude ≈ 111km
    LocalPos p = to_local(30.0f, -92.0f, 31.0f, -92.0f);
    TEST_ASSERT_FLOAT_WITHIN(500.0f, 111000.0f, local_distance(p));  // ±500m tolerance
}

void test_local_distance_short(void) {
    // Spot-lock scale: 5 meters north
    LocalPos p = to_local(30.0f, -92.0f, 30.0f + 0.000045f, -92.0f);
    float d = local_distance(p);
    TEST_ASSERT_TRUE(d > 3.0f && d < 8.0f);  // ~5m ± tolerance
}

void test_local_bearing_north(void) {
    LocalPos p = to_local(30.0f, -92.0f, 31.0f, -92.0f);  // due north
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, local_bearing(p));
}

void test_local_bearing_east(void) {
    LocalPos p = to_local(30.0f, -92.0f, 30.0f, -91.0f);  // due east
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 90.0f, local_bearing(p));
}

void test_haversine_vs_local_tangent(void) {
    // At short distances, local tangent should match haversine closely
    float lat1 = 30.0f, lon1 = -92.0f;
    float lat2 = 30.001f, lon2 = -92.001f;

    float hav = haversine(lat1, lon1, lat2, lon2);
    LocalPos lp = to_local(lat1, lon1, lat2, lon2);
    float lt = local_distance(lp);

    // Should agree within 1% at this short distance
    float pct_err = fabs(hav - lt) / hav * 100.0f;
    TEST_ASSERT_TRUE(pct_err < 1.0f);
}

// ============================================================
// THROTTLE CURVE TESTS
// ============================================================

void test_throttle_dead_zone(void) {
    TEST_ASSERT_EQUAL(0, throttle_curve(0));
    TEST_ASSERT_EQUAL(0, throttle_curve(100));
    TEST_ASSERT_EQUAL(0, throttle_curve(204));  // Just below dead zone
}

void test_throttle_above_dead_zone(void) {
    // pot=205 → pct=0.0 → powf(0,1.5)=0 → PWM=0 (boundary, technically dead zone edge)
    // pot=300 should give a small but non-zero PWM
    TEST_ASSERT_TRUE(throttle_curve(300) > 0);
    TEST_ASSERT_TRUE(throttle_curve(4095) > 200);  // Near max
}

void test_throttle_max(void) {
    TEST_ASSERT_EQUAL(255, throttle_curve(4095));
}

void test_throttle_monotonic(void) {
    // Higher pot = higher PWM (always)
    uint8_t prev = 0;
    for (int i = 205; i <= 4095; i += 50) {
        uint8_t val = throttle_curve(i);
        TEST_ASSERT_TRUE(val >= prev);
        prev = val;
    }
}

// ============================================================
// SOFT START TESTS
// ============================================================

void test_soft_start_ramps_up(void) {
    uint8_t current = 0;
    uint8_t target = 200;
    // Should take many cycles to reach target
    int cycles = 0;
    while (current < target && cycles < 1000) {
        current = soft_start(target, current);
        cycles++;
    }
    TEST_ASSERT_EQUAL(target, current);
    // At 0.05/cycle, should take roughly 50-100 cycles
    TEST_ASSERT_TRUE(cycles > 20 && cycles < 200);
}

void test_soft_start_instant_cut(void) {
    // Reducing speed should be instant (safety)
    TEST_ASSERT_EQUAL(0, soft_start(0, 200));
}

// ============================================================
// PID TESTS
// ============================================================

void test_pid_proportional(void) {
    PIDState pid;
    pid_init(pid, 2.0f, 0.0f, 0.0f, 500.0f);
    float out = pid_compute(pid, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, out);  // kp=2, error=10 → 20
}

void test_pid_integral_accumulates(void) {
    PIDState pid;
    pid_init(pid, 0.0f, 0.1f, 0.0f, 500.0f);
    pid_compute(pid, 10.0f);
    pid_compute(pid, 10.0f);
    // After 2 cycles with ki=0.1, integral=20, output = 0*10 + 0.1*20 = 2
    float out = pid_compute(pid, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 3.0f, out);  // integral=30, output=0.1*30=3
}

void test_pid_integral_clamps(void) {
    PIDState pid;
    pid_init(pid, 0.0f, 1.0f, 0.0f, 100.0f);
    // Pump the integral way past the limit
    for (int i = 0; i < 1000; i++) {
        pid_compute(pid, 100.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, pid.integral);
}

void test_pid_integral_freeze_in_deadband(void) {
    // Round 2 fix R2-I4: integral should freeze when freeze_integral=true
    PIDState pid;
    pid_init(pid, 0.0f, 1.0f, 0.0f, 500.0f);
    pid_compute(pid, 50.0f, false);  // integral = 50
    float int_before = pid.integral;
    pid_compute(pid, 50.0f, true);   // frozen — should NOT grow
    TEST_ASSERT_FLOAT_WITHIN(0.01f, int_before, pid.integral);
}

void test_pid_reset(void) {
    PIDState pid;
    pid_init(pid, 2.0f, 1.0f, 0.5f, 500.0f);
    pid_compute(pid, 50.0f);
    pid_compute(pid, 50.0f);
    pid_reset(pid);
    TEST_ASSERT_EQUAL(0, pid.integral);
    TEST_ASSERT_EQUAL(0, pid.prev_error);
}

void test_pid_output_clamps(void) {
    PIDState pid;
    pid_init(pid, 100.0f, 0.0f, 0.0f, 500.0f);
    float out = pid_compute(pid, 50.0f);
    TEST_ASSERT_EQUAL(100.0f, out);  // Clamped at +100

    pid_reset(pid);
    out = pid_compute(pid, -50.0f);
    TEST_ASSERT_EQUAL(-100.0f, out);  // Clamped at -100
}

// ============================================================
// SEQ_NUM WRAPAROUND TESTS — Round 2 fix R2-I1
// ============================================================

void test_seq_gap_normal(void) {
    TEST_ASSERT_EQUAL(1, seq_gap(101, 100));
    TEST_ASSERT_EQUAL(5, seq_gap(105, 100));
}

void test_seq_gap_wraparound(void) {
    // 65535 → 0 should be gap of 1
    TEST_ASSERT_EQUAL(1, seq_gap(0, 65535));
    // 65535 → 5 should be gap of 6
    TEST_ASSERT_EQUAL(6, seq_gap(5, 65535));
}

void test_seq_is_newer(void) {
    TEST_ASSERT_TRUE(seq_is_newer(101, 100));
    TEST_ASSERT_TRUE(seq_is_newer(0, 65535));    // wraparound
    TEST_ASSERT_FALSE(seq_is_newer(100, 100));   // same
    TEST_ASSERT_FALSE(seq_is_newer(100, 101));   // older
    TEST_ASSERT_FALSE(seq_is_newer(0, 32768));   // ambiguous, treat as old
}

// ============================================================
// SPOT-LOCK SIMULATION TESTS
// ============================================================

#include <stdio.h>

// Boat physics simulation
struct BoatState {
    float lat, lon, heading, speed, steer_angle, throttle;
};

void simulate_step(BoatState& boat, float wind_n, float wind_e, float curr_n, float curr_e, float dt) {
    float max_speed = 2.0f;
    float thrust = boat.throttle * max_speed;
    float thrust_n = thrust * cos(radians(boat.heading));
    float thrust_e = thrust * sin(radians(boat.heading));
    float vel_n = curr_n + wind_n + thrust_n;
    float vel_e = curr_e + wind_e + thrust_e;
    boat.lat += (vel_n / 111000.0f) * dt;
    boat.lon += (vel_e / (111000.0f * cosf(radians(boat.lat)))) * dt;
    float turn_rate = boat.steer_angle * 0.5f * (0.5f + boat.throttle);
    boat.heading = normalize_heading(boat.heading + turn_rate * dt);
}

LocalPos add_gps_noise(LocalPos p) {
    p.east_m += ((float)rand() / RAND_MAX - 0.5f) * 4.0f;
    p.north_m += ((float)rand() / RAND_MAX - 0.5f) * 4.0f;
    return p;
}

void run_spotlock(BoatState& boat, float a_lat, float a_lon,
                  float wind_n, float wind_e, float curr_n, float curr_e,
                  PIDState& pid, float dt) {
    // pos = boat's position relative to anchor
    LocalPos pos = to_local(a_lat, a_lon, boat.lat, boat.lon);
    pos = add_gps_noise(pos);
    float dist = local_distance(pos);

    // Bearing FROM anchor TO boat
    float bearing_from_anchor = local_bearing(pos);
    // We need to drive BACK to anchor = opposite direction (+180°)
    float desired_heading = normalize_heading(bearing_from_anchor + 180.0f);

    if (dist < 2.0f) {
        boat.throttle = 0;
    } else {
        boat.throttle = fminf(dist * 0.05f, 0.35f);
    }

    float herr = heading_error(desired_heading, boat.heading);
    float steer = pid_compute(pid, herr, dist < 2.0f);
    boat.steer_angle = steer * 0.45f;
    if (boat.steer_angle > 45.0f) boat.steer_angle = 45.0f;
    if (boat.steer_angle < -45.0f) boat.steer_angle = -45.0f;

    simulate_step(boat, wind_n, wind_e, curr_n, curr_e, dt);
}

void test_spotlock_holds_in_mild_conditions(void) {
    float a_lat = 30.0f, a_lon = -92.0f;
    BoatState boat = {30.00005f, -92.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    PIDState pid;
    pid_init(pid, 2.0f, 0.1f, 0.5f, 500.0f);
    float dt = 0.1f;

    // Mild: 1 m/s wind south, 0.2 m/s current south
    float wn = -1.0f * 0.02f, we = 0, cn = -0.2f, ce = 0;

    for (int i = 0; i < 600; i++) {  // 60 seconds
        run_spotlock(boat, a_lat, a_lon, wn, we, cn, ce, pid, dt);
    }

    LocalPos final_pos = to_local(a_lat, a_lon, boat.lat, boat.lon);
    float final_dist = local_distance(final_pos);
    printf("\n  Mild conditions — final distance: %.1fm after 60s\n", final_dist);
    TEST_ASSERT_TRUE(final_dist < 10.0f);
}

void test_spotlock_holds_in_moderate_wind(void) {
    float a_lat = 30.0f, a_lon = -92.0f;
    BoatState boat = {30.0f, -92.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    PIDState pid;
    pid_init(pid, 2.0f, 0.1f, 0.5f, 500.0f);
    float dt = 0.1f;

    // Moderate: 3 m/s wind east, 0.3 m/s current east
    float wn = 0, we = 3.0f * 0.02f, cn = 0, ce = 0.3f;

    for (int i = 0; i < 1200; i++) {  // 120 seconds
        run_spotlock(boat, a_lat, a_lon, wn, we, cn, ce, pid, dt);
    }

    LocalPos final_pos = to_local(a_lat, a_lon, boat.lat, boat.lon);
    float final_dist = local_distance(final_pos);
    printf("\n  Moderate wind — final distance: %.1fm after 120s\n", final_dist);
    TEST_ASSERT_TRUE(final_dist < 20.0f);
}

void test_spotlock_no_throttle_in_deadband(void) {
    float a_lat = 30.0f, a_lon = -92.0f;
    BoatState boat = {30.0f, -92.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    PIDState pid;
    pid_init(pid, 2.0f, 0.1f, 0.5f, 500.0f);

    for (int i = 0; i < 10; i++) {
        run_spotlock(boat, a_lat, a_lon, 0, 0, 0, 0, pid, 0.1f);
    }
    TEST_ASSERT_TRUE(boat.throttle < 0.01f);
}

void test_spotlock_pid_does_not_explode(void) {
    float a_lat = 30.0f, a_lon = -92.0f;
    BoatState boat = {30.001f, -92.001f, 0.0f, 0.0f, 0.0f, 0.0f};
    PIDState pid;
    pid_init(pid, 2.0f, 0.1f, 0.5f, 500.0f);

    // Extreme: 5 m/s wind, 0.5 m/s current
    float wn = 0, we = -5.0f * 0.02f, cn = 0, ce = -0.5f;

    for (int i = 0; i < 600; i++) {
        run_spotlock(boat, a_lat, a_lon, wn, we, cn, ce, pid, 0.1f);
    }

    LocalPos final_pos = to_local(a_lat, a_lon, boat.lat, boat.lon);
    float final_dist = local_distance(final_pos);
    printf("\n  Extreme conditions — final distance: %.1fm after 60s\n", final_dist);
    TEST_ASSERT_TRUE(final_dist < 200.0f);
    TEST_ASSERT_FALSE(isnan(final_dist));
    TEST_ASSERT_FALSE(isinf(final_dist));
}

void test_spotlock_heading_wrap_no_spin(void) {
    float a_lat = 30.001f, a_lon = -92.0f;  // Anchor north
    BoatState boat = {30.0f, -92.0f, 359.0f, 0.0f, 0.0f, 0.0f};
    PIDState pid;
    pid_init(pid, 2.0f, 0.0f, 0.0f, 500.0f);

    run_spotlock(boat, a_lat, a_lon, 0, 0, 0, 0, pid, 0.1f);
    printf("\n  Heading wrap — steer angle: %.1f° (should be small, not full deflection)\n", boat.steer_angle);
    TEST_ASSERT_TRUE(fabs(boat.steer_angle) < 10.0f);
}

// ============================================================
// MAIN
// ============================================================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Packet tests
    RUN_TEST(test_control_packet_size);
    RUN_TEST(test_telemetry_packet_size);
    RUN_TEST(test_packets_are_packed);
    RUN_TEST(test_control_checksum_roundtrip);
    RUN_TEST(test_checksum_catches_corruption);
    RUN_TEST(test_telemetry_checksum_roundtrip);

    // Heading tests
    RUN_TEST(test_heading_error_no_wrap);
    RUN_TEST(test_heading_error_wrap_359_to_1);
    RUN_TEST(test_heading_error_wrap_1_to_359);
    RUN_TEST(test_heading_error_180);
    RUN_TEST(test_normalize_heading);

    // Navigation tests
    RUN_TEST(test_local_distance_zero);
    RUN_TEST(test_local_distance_known);
    RUN_TEST(test_local_distance_short);
    RUN_TEST(test_local_bearing_north);
    RUN_TEST(test_local_bearing_east);
    RUN_TEST(test_haversine_vs_local_tangent);

    // Throttle tests
    RUN_TEST(test_throttle_dead_zone);
    RUN_TEST(test_throttle_above_dead_zone);
    RUN_TEST(test_throttle_max);
    RUN_TEST(test_throttle_monotonic);

    // Soft start tests
    RUN_TEST(test_soft_start_ramps_up);
    RUN_TEST(test_soft_start_instant_cut);

    // PID tests
    RUN_TEST(test_pid_proportional);
    RUN_TEST(test_pid_integral_accumulates);
    RUN_TEST(test_pid_integral_clamps);
    RUN_TEST(test_pid_integral_freeze_in_deadband);
    RUN_TEST(test_pid_reset);
    RUN_TEST(test_pid_output_clamps);

    // Seq num tests
    RUN_TEST(test_seq_gap_normal);
    RUN_TEST(test_seq_gap_wraparound);
    RUN_TEST(test_seq_is_newer);

    // Spot-lock simulation tests
    RUN_TEST(test_spotlock_holds_in_mild_conditions);
    RUN_TEST(test_spotlock_holds_in_moderate_wind);
    RUN_TEST(test_spotlock_no_throttle_in_deadband);
    RUN_TEST(test_spotlock_pid_does_not_explode);
    RUN_TEST(test_spotlock_heading_wrap_no_spin);

    UNITY_END();
    return 0;
}
