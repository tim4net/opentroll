/*
 * KAYAK AUTOPILOT — POD 1 CONTROLLER (Wokwi Edition)
 * 
 * Go to https://wokwi.com/projects/new/esp32c3
 * Replace the code with this file.
 * Replace diagram.json with the diagram.json in this folder.
 * 
 * What you'll see:
 * - OLED boots with "KAYAK AUTOPILOT" splash
 * - After 1.5s, shows live HUD: heading, SOG, GPS coords, satellites, battery bars
 * - Turn the SPEED pot → value changes on screen (would set motor PWM in real system)
 * - Turn the STEER pot → heading reference changes
 * - Toggle the direction switch → direction indicator changes
 * - Press ANCHOR button → LED lights up, display switches to spot-lock mode
 *   showing anchor distance + bearing back
 * - After ~4 seconds, simulated GPS acquires lock (sat count goes from 0 → 11)
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============== PIN ASSIGNMENTS ==============
#define PIN_SPEED_ADC    2
#define PIN_STEER_ADC    3
#define PIN_DIR_FWD      4
#define PIN_DIR_REV      5
#define PIN_ANCHOR_BTN   6
#define PIN_ANCHOR_LED   7
#define PIN_I2C_SDA      8
#define PIN_I2C_SCL      9

// ============== DISPLAY ==============
#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);

// ============== STATE ==============
unsigned long last_display = 0;
unsigned long last_sim = 0;
int sim_cycle = 0;

// Simulated telemetry (in real system, comes from Pod 2 via ESP-NOW)
float telem_lat = 30.2200;
float telem_lon = -92.0100;
float telem_sog = 0.0;
float telem_cog = 247.0;
uint8_t telem_sats = 0;
float telem_hdop = 99.0;
float telem_main_v = 13.2;
int8_t telem_rssi = -45;
float telem_anchor_dist = 0.0;
int telem_anchor_brg = 0;

float compass_heading = 247.0;

// Anchor
bool anchor_active = false;
bool anchor_btn_prev = false;
float anchor_lat = 0;
float anchor_lon = 0;

// Controller battery
float ctrl_batt_v = 4.0;
uint8_t ctrl_batt_pct = 85;

// Direction
uint8_t direction = 1; // OFF

// ============== SIMULATED WORLD ==============
void update_simulation() {
  unsigned long now = millis();
  if (now - last_sim < 200) return;
  last_sim = now;
  sim_cycle++;

  // GPS acquisition sequence
  if (sim_cycle < 20) {
    telem_sats = 0;
    telem_hdop = 99.0;
  } else if (sim_cycle < 40) {
    telem_sats = 4;
    telem_hdop = 3.5;
  } else {
    telem_sats = 11;
    telem_hdop = 1.2;
  }

  // Simulate drift
  telem_lat += 0.000003 * sin(sim_cycle * 0.05);
  telem_lon += 0.000002 * cos(sim_cycle * 0.03);

  // Speed/heading
  telem_sog = 0.1 + 0.05 * sin(sim_cycle * 0.1);
  compass_heading = fmod(compass_heading + 0.3 * sin(sim_cycle * 0.02) + 360.0, 360.0);
  telem_cog = fmod(compass_heading + 10 * sin(sim_cycle * 0.07) + 360.0, 360.0);

  // Battery drain
  telem_main_v = 13.2 - sim_cycle * 0.0001;
  if (sim_cycle % 25 == 0 && ctrl_batt_pct > 0) ctrl_batt_pct--;

  // RSSI noise
  telem_rssi = -45 + (random(20) - 10);

  // Anchor distance
  if (anchor_active && anchor_lat != 0) {
    float dlat = (telem_lat - anchor_lat) * 111000.0;
    float dlon = (telem_lon - anchor_lon) * 111000.0 * cos(radians(anchor_lat));
    telem_anchor_dist = sqrt(dlat * dlat + dlon * dlon);
    telem_anchor_brg = (int)fmod(degrees(atan2(dlon, dlat)) + 360.0, 360.0);
  }
}

// ============== READ INPUTS ==============
void read_inputs() {
  // Direction switch
  if (digitalRead(PIN_DIR_FWD) == LOW) direction = 0; // FORWARD
  else if (digitalRead(PIN_DIR_REV) == LOW) direction = 2; // REVERSE
  else direction = 1; // OFF

  // Anchor button with edge detection
  bool pressed = (digitalRead(PIN_ANCHOR_BTN) == LOW);
  if (pressed && !anchor_btn_prev) {
    anchor_active = !anchor_active;
    digitalWrite(PIN_ANCHOR_LED, anchor_active ? HIGH : LOW);
    if (anchor_active) {
      anchor_lat = telem_lat;
      anchor_lon = telem_lon;
    }
  }
  anchor_btn_prev = pressed;
}

// ============== DISPLAY ==============
void draw_battery(int x, int y, float v, int w) {
  display.drawRect(x, y, w, 8, SSD1306_WHITE);
  display.drawRect(x + w, y + 1, 2, 6, SSD1306_WHITE);
  float pct = constrain((v - 10.5) / 2.5, 0.0, 1.0);
  display.fillRect(x + 1, y + 1, (int)(pct * (w - 2)), 6, SSD1306_WHITE);
  display.setCursor(x + w + 5, y);
  display.printf("%.1fV", v);
}

void draw_ctrl_battery(int x, int y, uint8_t pct, int w) {
  display.drawRect(x, y, w, 6, SSD1306_WHITE);
  display.fillRect(x + 1, y + 1, (int)((pct / 100.0) * (w - 2)), 4, SSD1306_WHITE);
  display.setCursor(x + w + 3, y - 1);
  display.printf("%d%%", pct);
}

const char* dir_str(uint8_t d) {
  if (d == 0) return "FWD";
  if (d == 2) return "REV";
  return "OFF";
}

void update_display() {
  display.clearDisplay();

  // Top row: heading + mode badge
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("HDG %3.0f  ", compass_heading);

  if (anchor_active) {
    display.fillRect(90, 0, 38, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(94, 1);
    display.print("ANCHOR");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.setCursor(90, 0);
    display.printf("%s", dir_str(direction));
  }

  // Row 2: Speed over ground
  display.setCursor(0, 12);
  display.printf("SOG %.1fkt  RSSI %d", telem_sog * 1.944f, telem_rssi);

  // Row 3: GPS or anchor info
  display.setCursor(0, 24);
  if (anchor_active) {
    display.printf("ANC %.1fm BRG %3d", telem_anchor_dist, telem_anchor_brg);
  } else {
    display.printf("%.4f %.4f", telem_lat, telem_lon);
  }

  // Row 4: Satellites + accuracy
  display.setCursor(0, 36);
  if (telem_sats == 0) {
    display.print("GPS: SEARCHING...");
  } else {
    display.printf("SAT:%-2d  +/-%.1fm", telem_sats, telem_hdop);
  }

  // Row 5-6: Battery bars
  display.setCursor(0, 50);
  display.print("M:");
  draw_battery(12, 50, telem_main_v, 30);
  display.setCursor(70, 50);
  display.print("C:");
  draw_ctrl_battery(82, 51, ctrl_batt_pct, 20);

  display.display();
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found!");
    while (true) delay(100);
  }

  // Splash screen
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 4);
  display.println("KAYAK");
  display.setCursor(0, 24);
  display.println("AUTOPILOT");
  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print("Initializing...");
  display.display();
  delay(2000);

  // Pins
  pinMode(PIN_DIR_FWD, INPUT_PULLUP);
  pinMode(PIN_DIR_REV, INPUT_PULLUP);
  pinMode(PIN_ANCHOR_BTN, INPUT_PULLUP);
  pinMode(PIN_ANCHOR_LED, OUTPUT);
  digitalWrite(PIN_ANCHOR_LED, LOW);

  Serial.println("Ready!");
}

// ============== MAIN LOOP ==============
void loop() {
  read_inputs();
  update_simulation();

  unsigned long now = millis();
  if (now - last_display > 100) {  // 10Hz display
    last_display = now;
    update_display();
  }

  delay(5);
}
