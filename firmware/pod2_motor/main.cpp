/*
 * OpenTroll — Pod 2 Motor Brain (ESP32)
 *
 * Real-time motor controller. Receives ESP-NOW control packets,
 * drives propulsion H-bridge and steering actuator, runs spot-lock PID.
 *
 * This is the main application — not a library header.
 * Flash to ESP32 dev board with PlatformIO: pio run -e esp32 -t upload
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include "packets.h"
#include "navigation.h"
#include "control.h"

// ============== PIN ASSIGNMENTS ==============
#define RPWM_PIN     25   // IBT-2 forward PWM
#define LPWM_PIN     26   // IBT-2 reverse PWM
#define HB_EN_PIN    27   // IBT-2 enable (HIGH = enabled)
#define STEER_IN1    12   // DRV8871 steering actuator input 1
#define STEER_IN2    14   // DRV8871 steering actuator input 2
#define GPS_RX_PIN   16   // NEO-M8N TX → ESP32 RX2
#define GPS_TX_PIN   17   // ESP32 TX2 → NEO-M8N RX
#define BATT_SENSE   34   // ADC1_CH6 — battery voltage divider
#define STEER_SENSE  35   // ADC1_CH7 — AS5600 steering position (analog)
#define ESC_TEMP     36   // ADC1_CH0 — thermistor on IBT-2 heatsink
#define HB_PWM_FREQ  20000   // 20kHz — above audible
#define PWM_RES      8       // 8-bit (0-255)

// ============== STATE MACHINE ==============
enum SystemState {
  STATE_BOOT,
  STATE_POST,
  STATE_DISARMED,
  STATE_ARMING,
  STATE_MANUAL,
  STATE_SPOT_LOCK,
  STATE_RF_TIMEOUT,
  STATE_ERROR,
  STATE_ERROR_LOCKOUT
};

SystemState current_state = STATE_BOOT;
uint8_t error_code = ERR_OK;

// ============== RF / ESP-NOW ==============
// Double-buffered packet receive (Round 2 fix R2-C6)
static ControlPacket pkt_latest;
static portMUX_TYPE pkt_spinlock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool pkt_new = false;
static uint32_t last_packet_rx = 0;

// CS-10: only accept packets from the paired Pod 1 controller.
// Set to your Pod 1 MAC at pairing/flash time (real version loads from NVS).
// {0,0,0,0,0,0} = unpaired: REJECT EVERYTHING until pairing is configured —
// fail closed, not open.
static uint8_t paired_pod1_mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint16_t last_seq_num = 0;
static bool seq_initialized = false;
static uint32_t rejected_packets = 0;

// CS-10: range-check every field before use
static bool packet_fields_valid(const ControlPacket& p) {
  if (p.speed_raw > 4095) return false;
  if (p.steering_raw > 4095) return false;
  if (p.direction > DIR_REVERSE) return false;   // 0,1,2 only
  if (p.anchor_btn > 1) return false;
  if (p.heading_raw > 359) return false;
  if (p.ctrl_batt_pct > 100) return false;
  return true;
}

// ESP-NOW receive callback
void on_recv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(ControlPacket)) { rejected_packets++; return; }

  // Sender MAC filter (CS-10) — fail closed when unpaired
  if (memcmp(mac, paired_pod1_mac, 6) != 0) { rejected_packets++; return; }

  ControlPacket incoming;
  memcpy(&incoming, data, sizeof(ControlPacket));

  // Validate BEFORE accepting: checksum, field ranges, replay
  if (!verify_checksum(incoming)) { rejected_packets++; return; }
  if (!packet_fields_valid(incoming)) { rejected_packets++; return; }
  if (seq_initialized && !seq_is_newer(incoming.seq_num, last_seq_num)) {
    rejected_packets++;  // stale or replayed packet
    return;
  }
  last_seq_num = incoming.seq_num;
  seq_initialized = true;

  portENTER_CRITICAL(&pkt_spinlock);
  memcpy(&pkt_latest, &incoming, sizeof(ControlPacket));
  pkt_new = true;
  portEXIT_CRITICAL(&pkt_spinlock);
  last_packet_rx = millis();
}

ControlPacket get_latest_packet() {
  static ControlPacket working;
  portENTER_CRITICAL(&pkt_spinlock);
  memcpy(&working, &pkt_latest, sizeof(ControlPacket));
  pkt_new = false;
  portEXIT_CRITICAL(&pkt_spinlock);
  return working;
}

bool rf_link_active() {
  return (millis() - last_packet_rx) < 500;  // 500ms timeout
}

// ============== MOTOR SAFETY ==============
void kill_motor() {
  ledcWrite(0, 0);  // RPWM = 0
  ledcWrite(1, 0);  // LPWM = 0
  digitalWrite(HB_EN_PIN, LOW);
}

// PWM to zero but H-bridge stays enabled (dead-time windows, DIR_OFF)
void kill_motor_pwm() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void enable_hbridge() {
  digitalWrite(HB_EN_PIN, HIGH);
}

void disable_hbridge() {
  digitalWrite(HB_EN_PIN, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

// Single gate for all propulsion drive. Shoot-through prevention is
// structural: one code path, one direction at a time, opposite side always
// zeroed first with settle time.
//
// NOTE (honesty): firmware cannot DETECT hardware shoot-through. The old
// code read back ledcRead() after writing — that only reflects the duty
// registers we just wrote and can never observe gate/driver faults. Real
// shoot-through protection is electrical (the BTS7960 has interlocked
// half-bridges per side; dead-time here protects the motor/wiring on
// direction reversal). The fake check has been removed.
void drive_motor(uint8_t direction, uint8_t duty) {
  switch (direction) {
    case DIR_FORWARD:
      ledcWrite(1, 0);           // opposite side to 0 first
      delayMicroseconds(10);     // driver settle
      ledcWrite(0, duty);
      break;
    case DIR_REVERSE:
      ledcWrite(0, 0);
      delayMicroseconds(10);
      ledcWrite(1, duty);
      break;
    default:                     // DIR_OFF or anything unexpected
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      break;
  }
}

void drive_forward(uint8_t duty) { drive_motor(DIR_FORWARD, duty); }
void drive_reverse(uint8_t duty) { drive_motor(DIR_REVERSE, duty); }

// ============== GLOBALS ==============
uint8_t current_pwm = 0;
uint8_t current_direction = DIR_OFF;

// PID state
PIDState heading_pid;

// GPS (simplified — real implementation uses TinyGPS++ or similar)
float gps_lat = 0, gps_lon = 0, gps_sog = 0, gps_cog = 0;
uint8_t gps_sats = 0;
float gps_hdop = 99.0;
uint32_t gps_last_fix = 0;

// Spot-lock target
float anchor_lat = 0, anchor_lon = 0;
bool spot_lock_active = false;
AnchorDebounce anchor_db;
uint8_t anchor_prev_stable = 0;

// Battery
float battery_voltage = 13.2;
BattFilter batt_filter;
BattMonitor batt_monitor;

// ESC thermal
TempState esc_temp_state = TEMP_OK;
float esc_temp_c = 25.0f;

// Telemetry
uint32_t last_telem_tx = 0;
uint16_t telem_seq = 0;

// ============== SPOT-LOCK ==============
// CS-2: GPS-stale handling. Throttle to 0 immediately; if quality isn't
// restored within 10s, drop back to MANUAL (the operator regains control
// instead of the boat silently idling on a dead anchor).
static uint32_t gps_stale_since = 0;
// CS-3: heading source with hysteresis
static HeadingSource heading_source = HDG_COMPASS;
// Compass heading from Pod 1 (sent in every control packet)
static float remote_compass_heading = 0;

// Returns false when spot-lock should exit to MANUAL
bool spot_lock_loop() {
  bool gps_ok = (gps_sats >= 4) && (gps_hdop <= 4.0f) &&
                (millis() - gps_last_fix <= 2000);

  if (!gps_ok) {
    // GPS unreliable — cut throttle NOW, start/continue the 10s clock
    kill_motor_pwm();
    current_pwm = 0;               // soft-start must ramp from 0 on recovery
    error_code = ERR_GPS_STALE;
    if (gps_stale_since == 0) gps_stale_since = millis();
    if (millis() - gps_stale_since > 10000) {
      gps_stale_since = 0;
      return false;                // CS-2: exit to MANUAL after 10s
    }
    return true;
  }
  gps_stale_since = 0;

  // Position relative to anchor
  LocalPos pos = to_local(anchor_lat, anchor_lon, gps_lat, gps_lon);
  float dist = local_distance(pos);
  float bearing_from_anchor = local_bearing(pos);
  float desired_heading = normalize_heading(bearing_from_anchor + 180.0f);

  // CS-3: COG above 0.8 m/s, compass below 0.3 m/s, hysteresis between.
  // Reset PID derivative state on source switch to avoid a kick.
  HeadingSource new_source = select_heading_source(heading_source, gps_sog);
  if (new_source != heading_source) {
    heading_pid.prev_error = 0;
    heading_source = new_source;
  }
  float current_heading = (heading_source == HDG_COG) ? gps_cog
                                                      : remote_compass_heading;

  // Dead band
  if (dist < 2.0f) {
    kill_motor_pwm();
    current_pwm = 0;
    // Freeze integral in dead band
    float herr = heading_error(desired_heading, current_heading);
    pid_compute(heading_pid, herr, true);
    return true;
  }

  error_code = ERR_OK;

  // Throttle proportional to distance, capped at 35%
  float throttle_pct = fminf(dist * 0.05f, 0.35f);
  uint8_t target_pwm = (uint8_t)(throttle_pct * 255.0f);
  target_pwm = apply_limp_mode(target_pwm, batt_monitor.state);
  target_pwm = apply_temp_derate(target_pwm, esc_temp_state);
  target_pwm = soft_start(target_pwm, current_pwm);
  current_pwm = target_pwm;

  // Heading PID → steering
  float herr = heading_error(desired_heading, current_heading);
  float steer_cmd = pid_compute(heading_pid, herr, false);

  // Drive steering actuator
  // (steer_cmd: -100 to +100, map to DRV8871)
  // TODO: implement actuator drive with position feedback

  // Drive motor forward
  drive_forward(current_pwm);
  return true;
}

// ============== MANUAL MODE ==============
// CS-1: direction changes use NON-BLOCKING dead-time. The old code called
// delay(100), which froze the entire loop — RF watchdog, battery check,
// everything — for 100ms. Now we hold PWM at 0 and keep looping.
static uint32_t dead_time_until = 0;
static uint8_t pending_direction = DIR_OFF;

void manual_loop(ControlPacket& ctrl) {
  uint8_t target_pwm = throttle_curve(ctrl.speed_raw);
  // Limp mode: low battery caps throttle at 50% (spec CS-5)
  target_pwm = apply_limp_mode(target_pwm, batt_monitor.state);
  // Thermal derate: 50% cap at 85°C, kill at 90°C (README layer 8)
  target_pwm = apply_temp_derate(target_pwm, esc_temp_state);

  // Dead-time window active: hold motor at 0, keep the loop running
  if (dead_time_until != 0) {
    if ((int32_t)(millis() - dead_time_until) < 0) {
      kill_motor_pwm();          // PWM to 0, H-bridge stays enabled
      current_pwm = 0;
      return;
    }
    // Dead-time expired — adopt the pending direction at zero PWM
    current_direction = pending_direction;
    dead_time_until = 0;
    current_pwm = 0;
  }

  // Direction change between FWD and REV requires a 100ms dead-time
  if ((ctrl.direction == DIR_FORWARD && current_direction == DIR_REVERSE) ||
      (ctrl.direction == DIR_REVERSE && current_direction == DIR_FORWARD)) {
    kill_motor_pwm();
    current_pwm = 0;
    pending_direction = ctrl.direction;
    dead_time_until = millis() + 100;
    return;
  }

  target_pwm = soft_start(target_pwm, current_pwm);
  current_pwm = target_pwm;

  switch (ctrl.direction) {
    case DIR_FORWARD:
      drive_forward(current_pwm);
      current_direction = DIR_FORWARD;
      break;

    case DIR_REVERSE:
      drive_reverse(current_pwm);
      current_direction = DIR_REVERSE;
      break;

    case DIR_OFF:
    default:
      kill_motor_pwm();
      current_pwm = 0;
      current_direction = DIR_OFF;
      break;
  }

  // Steering: proportional actuator control from spring-return pot
  // Map ADC center (2048) ± deadzone to actuator drive
  // TODO: implement with AS5600 feedback
}

// ============== ESC THERMAL MONITORING ==============
// NOTE: spec v3 calls for a DS18B20 on GPIO4 (1-Wire); the original pin
// map here has an analog thermistor on GPIO36. Until that discrepancy is
// resolved in hardware, we read the analog pin (10k NTC divider assumed).
// Calibrate B-constant for your thermistor before trusting absolute values;
// thresholds err conservative.
float read_esc_temp() {
  // 10k NTC (B=3950) with 10k divider to 3.3V on ESC_TEMP
  uint16_t raw = analogRead(ESC_TEMP);
  if (raw == 0 || raw >= 4095) return esc_temp_c;  // open/short — hold last
  float v = (float)raw * 3.3f / 4095.0f;
  float r_ntc = 10000.0f * v / (3.3f - v);
  float t_k = 1.0f / (1.0f / 298.15f + logf(r_ntc / 10000.0f) / 3950.0f);
  return t_k - 273.15f;
}

void check_esc_temp() {
  esc_temp_c = read_esc_temp();
  TempState prev = esc_temp_state;
  esc_temp_state = temp_state_update(esc_temp_state, esc_temp_c);
  if (esc_temp_state == TEMP_KILL && prev != TEMP_KILL) {
    kill_motor();
    error_code = ERR_ESC_OVERTEMP;
    if (current_state == STATE_MANUAL || current_state == STATE_SPOT_LOCK) {
      current_state = STATE_ERROR;
    }
  }
  // Recoverable per CS-6: cooled below 75°C → DISARMED (manual re-arm)
  if (prev == TEMP_KILL && esc_temp_state == TEMP_OK &&
      current_state == STATE_ERROR && error_code == ERR_ESC_OVERTEMP) {
    current_state = STATE_DISARMED;
    disable_hbridge();
  }
}

// ============== BATTERY MONITORING ==============
float read_battery_voltage() {
  // Voltage divider: 120kΩ (battery+) to 30kΩ (GND) on GPIO34 (ADC1) — spec v3.
  // DO NOT revert to 100k/33k: that puts 3.62V on the ADC at 14.6V full charge.
  uint16_t raw = analogRead(BATT_SENSE);
  return batt_filter_update(batt_filter, adc_to_battery_voltage(raw));
}

BattState check_battery() {
  battery_voltage = read_battery_voltage();
  BattState bs = batt_monitor_update(batt_monitor, battery_voltage, millis());
  if (bs == BATT_CRITICAL)      error_code = ERR_CRIT_BATTERY;
  else if (bs == BATT_LOW)      error_code = ERR_LOW_BATTERY;
  return bs;
}

// ============== TELEMETRY ==============
void send_telemetry() {
  TelemetryPacket tp;
  memset(&tp, 0, sizeof(tp));
  tp.device_id = 1;
  tp.fw_version = 1;
  tp.seq_num = ++telem_seq;
  tp.lat = gps_lat;
  tp.lon = gps_lon;
  tp.sog = gps_sog;
  tp.cog = gps_cog;
  tp.satellites = gps_sats;
  tp.hdop = gps_hdop;
  tp.main_voltage = battery_voltage;
  tp.motor_mode = (current_state == STATE_SPOT_LOCK) ? 1 : 0;
  tp.steer_angle = 0;  // TODO: from AS5600
  tp.anchor_dist = 0;  // TODO: computed in spot_lock
  tp.anchor_brg = 0;
  tp.error_code = error_code;
  tp.rssi = (int8_t)WiFi.RSSI();
  tp.uptime_ms = millis();
  set_checksum(tp);

  // Send to Pod 1 via ESP-NOW
  // esp_now_send(peer_addr, (uint8_t*)&tp, sizeof(tp));
}

// ============== POST (Power-On Self-Test) ==============
bool run_post() {
  // Prime the battery filter with a full window, then check
  batt_filter_init(batt_filter);
  batt_monitor_init(batt_monitor);
  for (int i = 0; i < BATT_FILTER_LEN; i++) read_battery_voltage();
  if (check_battery() == BATT_CRITICAL) return false;

  // Check reset reason — require re-arm after watchdog/brownout
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason == ESP_RST_WDT || reason == ESP_RST_BROWNOUT) {
    error_code = ERR_WATCHDOG;
    return false;
  }

  // GPS will acquire in background — don't block boot
  // Compass check would go here

  return true;
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  Serial.println("OpenTroll Pod 2 — Motor Brain");

  // CRITICAL: Set PWM pins LOW before anything else (boot-state safety)
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(HB_EN_PIN, OUTPUT);
  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);
  digitalWrite(HB_EN_PIN, LOW);  // H-bridge disabled at boot

  // PWM init (20kHz, 8-bit)
  ledcSetup(0, HB_PWM_FREQ, PWM_RES);  // Channel 0 = RPWM
  ledcSetup(1, HB_PWM_FREQ, PWM_RES);  // Channel 1 = LPWM
  ledcAttachPin(RPWM_PIN, 0);
  ledcAttachPin(LPWM_PIN, 1);
  ledcWrite(0, 0);
  ledcWrite(1, 0);

  // ADC
  analogReadResolution(12);

  // PID init
  pid_init(heading_pid, 2.0f, 0.1f, 0.5f, 500.0f);
  anchor_debounce_init(anchor_db);

  // ESP-NOW init (simplified — real version loads peer + keys from NVS)
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(on_recv);
  // Register paired Pod 1 as the only peer. When ESP-NOW encryption is
  // enabled (LMK/PMK set at pairing), packets from unknown/unencrypted
  // senders are dropped by the radio; the MAC filter in on_recv is the
  // software backstop.
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, paired_pod1_mac, 6);
  peer.channel = 0;
  peer.encrypt = false;  // TODO(pairing): set true + LMK once NVS pairing lands
  esp_now_add_peer(&peer);

  // Hardware watchdog (CS-7): if the loop hangs, reset. POST checks the
  // reset reason and refuses to auto-arm after a watchdog reset.
  // 500ms per spec; panic=true forces reset on starvation.
  esp_task_wdt_init(1, true);   // 1s is the API floor; spec intent preserved
  esp_task_wdt_add(NULL);       // watch the loop task

  // POST
  if (run_post()) {
    current_state = STATE_DISARMED;
    Serial.println("POST passed. Waiting for RF link...");
  } else {
    current_state = STATE_ERROR_LOCKOUT;
    Serial.println("POST FAILED. Lockout.");
  }
}

// ============== MAIN LOOP (20Hz) ==============
void loop() {
  // 0. Feed the hardware watchdog (CS-7) — if this loop ever hangs,
  // the WDT resets the chip and POST blocks auto-rearm.
  esp_task_wdt_reset();

  // 1. RF watchdog
  if (current_state != STATE_BOOT && current_state != STATE_POST &&
      current_state != STATE_ERROR_LOCKOUT) {
    if (!rf_link_active()) {
      current_state = STATE_RF_TIMEOUT;
      kill_motor();
    }
  }

  // 2. Battery check (every cycle) — hysteresis + latched critical (spec CS-5)
  BattState batt_state = check_battery();
  if (batt_state == BATT_CRITICAL &&
      current_state != STATE_ERROR && current_state != STATE_ERROR_LOCKOUT) {
    kill_motor();
    current_state = STATE_ERROR;
    error_code = ERR_CRIT_BATTERY;
  }
  // Recovery: BattMonitor only leaves CRITICAL after >11.5V sustained 2s.
  // Go to DISARMED (manual re-arm), never straight back to drive.
  if (current_state == STATE_ERROR && error_code == ERR_CRIT_BATTERY &&
      batt_state != BATT_CRITICAL) {
    current_state = STATE_DISARMED;
    disable_hbridge();
  }

  // 2b. ESC thermal check (1Hz — thermal time constants are slow)
  static uint32_t last_temp_check = 0;
  if (millis() - last_temp_check >= 1000) {
    check_esc_temp();
    last_temp_check = millis();
  }

  // 3. State machine
  switch (current_state) {
    case STATE_DISARMED: {
      // CS-4 + zero-throttle interlock. Arming requires ALL of:
      //   - 2s of CONTINUOUS RF (any gap >100ms resets the timer)
      //   - direction switch at OFF
      //   - speed dial at zero (below throttle dead zone)
      // Without the throttle interlock, an RF drop + recovery would relaunch
      // the boat at whatever the speed dial was left at.
      static uint32_t arm_timer = 0;

      if (!rf_link_active()) {
        arm_timer = 0;
        break;
      }
      // Any RF gap > 100ms during arming resets the timer (CS-4)
      if (millis() - last_packet_rx > 100) {
        arm_timer = 0;
      }

      ControlPacket ctrl = get_latest_packet();
      if (!verify_checksum(ctrl) ||
          ctrl.direction != DIR_OFF ||
          throttle_curve(ctrl.speed_raw) != 0) {
        arm_timer = 0;   // controls not safe — restart the clock
        break;
      }

      if (arm_timer == 0) arm_timer = millis();
      if (millis() - arm_timer > 2000) {
        enable_hbridge();
        current_state = STATE_MANUAL;
        current_pwm = 0;
        current_direction = DIR_OFF;
        Serial.println("ARMED — Manual mode (throttle zero, direction OFF)");
        arm_timer = 0;
      }
      break;
    }

    case STATE_MANUAL: {
      if (!rf_link_active()) break;
      ControlPacket ctrl = get_latest_packet();
      if (!verify_checksum(ctrl)) break;
      remote_compass_heading = (float)ctrl.heading_raw;

      // Spot-lock engagement: debounced rising edge (CS-11) + GPS quality
      uint8_t anchor_stable = anchor_debounce(anchor_db, ctrl.anchor_btn);
      if (anchor_stable && !anchor_prev_stable && !spot_lock_active &&
          gps_sats >= 4) {
        anchor_lat = gps_lat;
        anchor_lon = gps_lon;
        spot_lock_active = true;
        pid_reset(heading_pid);
        current_state = STATE_SPOT_LOCK;
        Serial.println("SPOT-LOCK engaged");
      } else {
        manual_loop(ctrl);
      }
      anchor_prev_stable = anchor_stable;
      break;
    }

    case STATE_SPOT_LOCK: {
      // Check panic override
      if (rf_link_active()) {
        ControlPacket ctrl = get_latest_packet();
        if (verify_checksum(ctrl)) {
          remote_compass_heading = (float)ctrl.heading_raw;
          uint16_t center = 2048;
          uint16_t deadzone = 100;
          if (abs(ctrl.steering_raw - center) > deadzone ||
              ctrl.direction != DIR_FORWARD) {
            // Panic override — back to manual
            spot_lock_active = false;
            current_state = STATE_MANUAL;
            Serial.println("Panic override — back to manual");
            break;
          }
          // Anchor button toggle off (debounced)
          if (anchor_debounce(anchor_db, ctrl.anchor_btn) == 0) {
            spot_lock_active = false;
            current_state = STATE_MANUAL;
            Serial.println("Spot-lock disengaged");
            break;
          }
        }
      }

      // CS-2: spot_lock_loop returns false when GPS has been stale >10s
      if (!spot_lock_loop()) {
        spot_lock_active = false;
        current_pwm = 0;
        current_state = STATE_MANUAL;
        Serial.println("GPS lost >10s — spot-lock exit to manual");
      }
      break;
    }

    case STATE_RF_TIMEOUT: {
      if (rf_link_active()) {
        // Require manual re-arm
        current_state = STATE_DISARMED;
      }
      break;
    }

    case STATE_ERROR: {
      kill_motor();
      // Require power cycle
      break;
    }

    case STATE_ERROR_LOCKOUT: {
      kill_motor();
      break;
    }

    default:
      break;
  }

  // 4. Send telemetry (10Hz)
  if (millis() - last_telem_tx > 100) {
    send_telemetry();
    last_telem_tx = millis();
  }

  // 5. Loop timing (~20Hz = 50ms)
  delay(50);
}
