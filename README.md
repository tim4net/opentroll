# OpenTroll

> Open-source wireless autopilot for trolling motors — GPS spot-lock, remote steering, and autonomous navigation for kayaks and small boats.

**OpenTroll** is a complete DIY system that adds wireless remote control and GPS anchoring (spot-lock) to any cheap trolling motor. No $1,500 factory system required. Build it for under $150 in electronics.

```
The gap OpenTroll fills:

  $100    PEXMOR / cheap motor (manual only)
  $150    OpenTroll Basic Kit (wireless remote + steering)     ← YOU ARE HERE
  $300    OpenTroll GPS Kit (+ spot-lock, routes, autopilot)
   |
   ──────── $1,000+ gap with no existing product ────────
   |
  $1,300  Minn Kota Terrova + i-Pilot (factory, closed)
  $1,500  MotorGuide Xi3 GPS (factory, closed)
```

## ✨ Features

- **Wireless remote control** — Speed and steering from a handheld controller, no deck cables
- **GPS Spot-Lock** — Virtual anchor. Holds your position automatically using GPS + compass
- **Tap-to-Navigate** — Tap a point on the map, the boat drives there and anchors (v2)
- **Works with any motor** — Minn Kota, PEXMOR, Haswing, any brushed DC trolling motor
- **Sub-4ms latency** — ESP-NOW wireless protocol, not WiFi or Bluetooth
- **Open firmware** — Full PID control loop, safety systems, configurable parameters
- **Physical dials** — Operate by muscle memory with wet hands. No touchscreen required.

## 🏗️ Architecture

```
┌──────────────────────┐                    ┌──────────────────────────┐
│   POD 1              │    ESP-NOW         │   POD 2                  │
│   Hip Controller     │    Auth + CRC      │   Stern Motor Brain      │
│                      │    50-100 Hz       │                          │
│  ┌────────────────┐  │   ◄────────────►   │  ┌────────────────────┐  │
│  │ ESP32-C3       │  │                    │  │ ESP32              │  │
│  │                │  │  Telemetry 10Hz    │  │                    │  │
│  │ Speed Dial     │  │   ◄────────────►   │  │ IBT-2 H-Bridge     │  │
│  │ Steering Dial  │  │                    │  │ (Motor PWM)        │  │
│  │ Dir Switch     │  │                    │  │                    │  │
│  │ Anchor Button  │  │                    │  │ DRV8871            │  │
│  │ OLED Display   │  │                    │  │ (Steering Actuator)│  │
│  │ Compass        │  │                    │  │                    │  │
│  │ 18650 Battery  │  │                    │  │ NEO-M8N GPS        │  │
│  └────────────────┘  │                    │  │ Buck Converter     │  │
│                      │                    │  │ Kill Switch Relay  │  │
│  Clear IP67 Box      │                    │  └────────────────────┘  │
│  Mounts on gear track│                    │  IP67 Dry Box            │
└──────────────────────┘                    │  Mounts near motor       │
                                            └──────────────────────────┘
                                                         │
                                                    8 AWG wire
                                                         │
                                                    ▼
                                              ┌─────────────┐
                                              │ Trolling    │
                                              │ Motor       │
                                              │ (any brand) │
                                              └─────────────┘
```

### Pod 1 — Controller (Transmitter)

Mounts at hip height on the kayak gear track. All-physical controls — no looking down, no touchscreen.

| Component | Purpose |
|-----------|---------|
| ESP32-C3 Super Mini | Wireless transmitter, reads all inputs |
| Speed potentiometer | Cruise throttle — turns smoothly, holds position |
| Spring-return steering pot | Steering — snaps back to center when released |
| ON-OFF-ON toggle switch | Forward / Off / Reverse |
| IP67 LED button | Spot-lock toggle (glows blue when anchored) |
| 0.96" OLED display | Heading, GPS, battery, anchor distance |
| GY-271 compass | Heading reference |
| 18650 + USB-C | 30hr runtime, rechargeable |

### Pod 2 — Motor Brain (Receiver)

Mounts at the stern inside a waterproof box, close to the motor.

| Component | Purpose |
|-----------|---------|
| ESP32 | Real-time motor controller |
| IBT-2 (BTS7960) 43A | H-bridge drives the propeller |
| DRV8871 | H-bridge drives the steering actuator |
| NEO-M8N GPS | 10Hz position for spot-lock |
| High-speed linear actuator | Physically steers the motor head |
| AS5600 encoder | Steering position feedback for PID |
| Latching kill switch relay | Lanyard cutoff — hardware power shutoff |

## 🛡️ Safety

OpenTroll is designed with **layered safety** — no single point of failure can cause a runaway boat.

**Status legend:** ✅ implemented in firmware · 🔩 hardware/electrical · 🚧 planned (not yet in firmware — do not rely on it)

| Layer | What | How | Status |
|-------|------|-----|--------|
| 1 | Lanyard kill switch | Hardware contactor cuts all motor power. Clips to PFD. Wire-break = fail-safe (motor stops). | 🔩 |
| 2 | Direction switch | Center position = motor OFF (mechanical) | 🔩 |
| 3 | RF timeout | No valid packet for 500ms → motor stops | ✅ |
| 4 | Panic override | Touch steering dial during spot-lock → instant manual mode | ✅ |
| 5 | H-bridge enable | GPIO disable, defaults OFF at boot | ✅ 🔩 |
| 6 | Shoot-through guard | **Hardware** interlock (BTS7960 half-bridges) + firmware single-gate drive with dead-time on reversal. Firmware cannot *detect* shoot-through — protection is structural + electrical. | ✅ 🔩 |
| 7 | Low-voltage cutoff | Battery below 10.5V → motor stops; latched with 11.5V/2s recovery + 50% limp mode | ✅ |
| 8 | Overtemperature | ESC: warn 80°C, derate 85°C, kill 90°C, recover below 75°C | ✅ |
| 9 | Boot-state protection | Pull-down resistors ensure motor is OFF during ESP32 boot | ✅ 🔩 |
| 10 | Power-on self-test | Battery + reset-reason checks; refuses auto-arm after watchdog/brownout | ✅ |
| 11 | Arming interlock | Arms only after 2s continuous RF **with throttle at zero and direction OFF** | ✅ |
| 12 | Hardware watchdog | Hung control loop → chip reset → stays disarmed | ✅ |
| 13 | RF authentication | Paired-MAC filter + CRC + replay rejection. ESP-NOW AES pairing 🚧 | ✅ (MAC/replay) / 🚧 (AES) |
| 14 | GPS-loss handling | Spot-lock cuts throttle on stale GPS, returns control to operator after 10s | ✅ |

**Honest status note:** Pod 2 (motor brain) firmware implements the safety
logic above. Still **planned / not yet implemented** and required before any
real on-water use: GPS NMEA parsing, steering-actuator drive with AS5600
feedback (spot-lock steers open-loop until then), telemetry TX, ESP-NOW AES
encryption/pairing, and the Pod 1 controller transmit firmware (the current
`pod1_controller/main.cpp` is the Wokwi **display simulator**, not a real
transmitter). Do not put this on water expecting autonomous spot-lock yet.

## 🧠 Spot-Lock (GPS Anchor)

The killer feature. Press the anchor button and OpenTroll captures your GPS position. A cascaded PID controller holds you there:

1. **Outer loop (10Hz):** GPS position error → desired heading
2. **Inner loop (20Hz):** Heading error → steering actuator
3. **Throttle:** Proportional to distance from anchor, capped for safety

Key algorithm details:
- **2m dead band** — no correction within 2m of anchor (avoids GPS jitter oscillation)
- **Moving-average GPS filter** — smooths 2-5m GPS noise
- **Heading wraparound-safe math** — won't spin in circles near magnetic north
- **Dual heading source** — GPS course-over-ground when moving, compass when stationary
- **PID anti-windup** — integral freezes in dead band to prevent oscillation

## 📟 Display

Pod 1's OLED shows a live HUD:

```
┌────────────────────────────┐
│ HDG 247   FWD              │
│ SOG 1.2kt  RSSI -48        │
│ 30.2200  -92.0100          │
│ SAT:11  ±1.2m              │
│ M:[#######] 13.1V  C:[84%] │
└────────────────────────────┘
```

When spot-lock is active:

```
┌────────────────────────────┐
│ HDG 012    ▓▓ ANCHOR ▓▓    │
│ SOG 0.3kt  RSSI -52        │
│ ANC: 3.2m  BRG  045        │
│ SAT:11  ±1.2m              │
│ M:[######] 12.9V  C:[82%]  │
└────────────────────────────┘
```

## 🗺️ Roadmap

### v1 — Core System (In Progress)
- [x] Firmware architecture designed
- [x] Packet protocol + safety systems specified
- [x] PID spot-lock algorithm + simulation (37 unit tests passing)
- [ ] ESP-NOW pairing + encrypted communication
- [ ] Manual drive mode (speed + steering + direction)
- [ ] Spot-lock mode (GPS anchor)
- [ ] Physical build + on-water testing

### v2 — Navigation + Display
- [ ] 5" sunlight-readable touchscreen
- [ ] Offline maps (Leaflet.js + cached tiles)
- [ ] Tap-to-navigate (tap map → autopilot drives there)
- [ ] Fishing spot database (SQLite + GPS pins + catch logs)
- [ ] Bathymetry mapping (depth logging over multiple trips)
- [ ] PID tuning UI
- [ ] Data logger / trip replay
- [ ] OTA firmware updates

### v3 — Advanced
- [ ] Open-source sonar integration ([Open Echo TUSS4470](https://github.com/Neumi/open_echo))
- [ ] Live echogram on display
- [ ] ML-assisted station keeping (Vanchor integration)
- [ ] LoRa mesh for spot sharing between boats
- [ ] Route recording + playback
- [ ] Return-to-launch / autonomous docking

## 🔧 What You Need

**Works with any brushed DC trolling motor** — Minn Kota, PEXMOR, Haswing, Watersnake, etc. If it has a permanent magnet DC motor (and almost all trolling motors under $500 do), OpenTroll can control it.

The only motor modification: open the tiller head, find the two wires going down the shaft, connect the H-bridge output there instead of the factory speed control. Reversible — restore factory wiring anytime.

**Tools required:**
- Soldering iron (basic through-hole soldering)
- Wire strippers + crimp tool
- Multimeter
- Drill (for enclosure holes)
- No 3D printer required — commercial waterproof boxes only

**Electronics knowledge:** If you can follow a wiring diagram and upload Arduino code, you can build this.

## 💻 Software Stack

| Layer | Technology | License |
|-------|-----------|---------|
| Pod 1 firmware (controller) | C++ / Arduino / ESP32-C3 | GPL-3.0 |
| Pod 2 firmware (motor brain) | C++ / Arduino / ESP32 | GPL-3.0 |
| Navigation library | C++ (header-only, no dependencies) | GPL-3.0 |
| Unit tests | PlatformIO + Unity | GPL-3.0 |
| Display simulator | Wokwi (browser) | — |
| v2 Web dashboard | Python / Flask / Leaflet.js (future) | GPL-3.0 |

## 🤝 Contributing

Contributions welcome. This is a Free Software project — all firmware, algorithms, and documentation are GPL-3.0.

Areas where help is needed:
- ESP32 ESP-NOW encryption + pairing protocol
- PID tuning on different boats/motors
- Mechanical bracket designs for popular motors
- Documentation and build guides
- Wokwi simulation models

## 📜 License

GPL-3.0-or-later. See [LICENSE](LICENSE).

Firmware you write for your own motor is yours. If you distribute it or sell a product based on OpenTroll, you must release your modifications under the same license.

## 🔗 Related Projects

- [Vanchor](https://github.com/AlexAsplund/Vanchor) — ML-assisted GPS anchor for boats (MIT)
- [Open Echo](https://github.com/Neumi/open_echo) — Open-source sonar/depth sounder (MIT)
- [ArduPilot Rover](https://ardupilot.org/rover/) — Autonomous boat/rover firmware (GPLv3)

---

*OpenTroll is built by fishermen, for fishermen. No subscriptions, no cloud, no app required. Your boat, your motor, your code.*
