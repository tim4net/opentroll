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

// ESP-NOW receive callback
void on_recv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(ControlPacket)) return;
  portENTER_CRITICAL(&pkt_spinlock);
  memcpy(&pkt_latest, data, sizeof(ControlPacket));
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

void enable_hbridge() {
  digitalWrite(HB_EN_PIN, HIGH);
}

void disable_hbridge() {
  digitalWrite(HB_EN_PIN, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void drive_forward(uint8_t duty) {
  // GUARANTEE: LPWM is 0 before RPWM goes high
  ledcWrite(1, 0);
  delayMicroseconds(10);
  ledcWrite(0, duty);

  // Firmware assertion — shoot-through check
  if (ledcRead(0) > 0 && ledcRead(1) > 0) {
    kill_motor();
    current_state = STATE_ERROR;
    error_code = ERR_SHOOT_THROUGH;
  }
}

void drive_reverse(uint8_t duty) {
  ledcWrite(0, 0);
  delayMicroseconds(10);
  ledcWrite(1, duty);

  if (ledcRead(0) > 0 && ledcRead(1) > 0) {
    kill_motor();
    current_state = STATE_ERROR;
    error_code = ERR_SHOOT_THROUGH;
  }
}

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

// Battery
float battery_voltage = 13.2;

// Telemetry
uint32_t last_telem_tx = 0;
uint16_t telem_seq = 0;

// ============== SPOT-LOCK ==============
void spot_lock_loop() {
  if (gps_sats < 4 || gps_hdop > 4.0) {
    // GPS unreliable — throttle to 0, warn
    drive_forward(0);
    error_code = ERR_GPS_STALE;
    return;
  }

  // Position relative to anchor
  LocalPos pos = to_local(anchor_lat, anchor_lon, gps_lat, gps_lon);
  float dist = local_distance(pos);
  float bearing_from_anchor = local_bearing(pos);
  float desired_heading = normalize_heading(bearing_from_anchor + 180.0f);

  // Dead band
  if (dist < 2.0f) {
    drive_forward(0);
    // Freeze integral in dead band
    float herr = heading_error(desired_heading, gps_cog);
    pid_compute(heading_pid, herr, true);
    return;
  }

  error_code = ERR_OK;

  // Throttle proportional to distance, capped at 35%
  float throttle_pct = fminf(dist * 0.05f, 0.35f);
  uint8_t target_pwm = (uint8_t)(throttle_pct * 255.0f);
  target_pwm = soft_start(target_pwm, current_pwm);
  current_pwm = target_pwm;

  // Heading PID → steering
  float current_heading = gps_cog;  // Use COG when moving
  float herr = heading_error(desired_heading, current_heading);
  float steer_cmd = pid_compute(heading_pid, herr, false);

  // Drive steering actuator
  // (steer_cmd: -100 to +100, map to DRV8871)
  // TODO: implement actuator drive with position feedback

  // Drive motor forward
  drive_forward(current_pwm);
}

// ============== MANUAL MODE ==============
void manual_loop(ControlPacket& ctrl) {
  uint8_t target_pwm = throttle_curve(ctrl.speed_raw);
  target_pwm = soft_start(target_pwm, current_pwm);
  current_pwm = target_pwm;

  switch (ctrl.direction) {
    case DIR_FORWARD:
      if (current_direction == DIR_REVERSE) {
        // Dead-time on direction change
        drive_forward(0);
        current_pwm = 0;
        delay(100);
      }
      drive_forward(current_pwm);
      current_direction = DIR_FORWARD;
      break;

    case DIR_REVERSE:
      if (current_direction == DIR_FORWARD) {
        drive_reverse(0);
        current_pwm = 0;
        delay(100);
      }
      drive_reverse(current_pwm);
      current_direction = DIR_REVERSE;
      break;

    case DIR_OFF:
    default:
      drive_forward(0);
      current_pwm = 0;
      current_direction = DIR_OFF;
      break;
  }

  // Steering: proportional actuator control from spring-return pot
  // Map ADC center (2048) ± deadzone to actuator drive
  // TODO: implement with AS5600 feedback
}

// ============== BATTERY MONITORING ==============
float read_battery_voltage() {
  // Voltage divider: 100kΩ + 33kΩ on GPIO34 (ADC1)
  // ADC reading × 4.03 × 3.3 / 4095 = battery voltage
  uint16_t raw = analogRead(BATT_SENSE);
  float v = (float)raw * 4.03f * 3.3f / 4095.0f;
  return v;
}

bool check_battery() {
  battery_voltage = read_battery_voltage();
  if (battery_voltage < 10.5f) {
    error_code = ERR_CRIT_BATTERY;
    return false;
  }
  if (battery_voltage < 11.5f) {
    error_code = ERR_LOW_BATTERY;
  }
  return true;
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
  // Check battery
  if (!check_battery()) return false;

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

  // ESP-NOW init (simplified — real version loads peer from NVS)
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(on_recv);

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
  // 1. RF watchdog
  if (current_state != STATE_BOOT && current_state != STATE_POST &&
      current_state != STATE_ERROR_LOCKOUT) {
    if (!rf_link_active()) {
      current_state = STATE_RF_TIMEOUT;
      kill_motor();
    }
  }

  // 2. Battery check (every cycle)
  check_battery();
  if (battery_voltage < 10.5f && current_state != STATE_ERROR_LOCKOUT) {
    kill_motor();
    current_state = STATE_ERROR;
    error_code = ERR_CRIT_BATTERY;
  }

  // 3. State machine
  switch (current_state) {
    case STATE_DISARMED: {
      // Wait for RF link, then arm with 2s delay
      if (rf_link_active()) {
        static uint32_t arm_timer = 0;
        if (arm_timer == 0) arm_timer = millis();
        if (millis() - arm_timer > 2000) {
          enable_hbridge();
          current_state = STATE_MANUAL;
          Serial.println("ARMED — Manual mode");
          arm_timer = 0;
        }
      }
      break;
    }

    case STATE_MANUAL: {
      if (!rf_link_active()) break;
      ControlPacket ctrl = get_latest_packet();
      if (!verify_checksum(ctrl)) break;

      // Check for spot-lock activation
      if (ctrl.anchor_btn && !spot_lock_active && gps_sats >= 4) {
        anchor_lat = gps_lat;
        anchor_lon = gps_lon;
        spot_lock_active = true;
        pid_reset(heading_pid);
        current_state = STATE_SPOT_LOCK;
        Serial.println("SPOT-LOCK engaged");
      } else {
        manual_loop(ctrl);
      }
      break;
    }

    case STATE_SPOT_LOCK: {
      // Check panic override
      if (rf_link_active()) {
        ControlPacket ctrl = get_latest_packet();
        if (verify_checksum(ctrl)) {
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
        }
      }

      // Check for anchor button toggle off
      if (rf_link_active()) {
        ControlPacket ctrl = get_latest_packet();
        if (verify_checksum(ctrl) && !ctrl.anchor_btn) {
          spot_lock_active = false;
          current_state = STATE_MANUAL;
          Serial.println("Spot-lock disengaged");
          break;
        }
      }

      spot_lock_loop();
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
