# OpenTroll Firmware

Open-source firmware for the OpenTroll wireless trolling motor autopilot.

## Structure

```
firmware/
├── src/                    # Shared library (header-only)
│   ├── packets.h           # Packed ESP-NOW packet structs + CRC16
│   ├── navigation.h        # Heading math, local tangent plane, haversine
│   └── control.h           # PID controller, throttle curve, soft-start, seq_num
├── pod1_controller/        # Pod 1 — ESP32-C3 hip controller
│   └── main.cpp            # Reads dials/buttons, drives OLED, sends ESP-NOW
├── pod2_motor/             # Pod 2 — ESP32 motor brain
│   └── main.cpp            # Receives commands, drives H-brides, spot-lock PID
├── test/                   # Unit tests
│   └── test_main.cpp       # 37 tests: packets, PID, spot-lock simulation
├── wokwi/                  # Browser simulator
│   ├── sketch.ino          # Paste into wokwi.com for visual demo
│   └── diagram.json        # Virtual breadboard layout
└── platformio.ini          # Build configs (native + esp32 + esp32c3)
```

## Quick Start

### Run tests (no hardware needed)

```bash
pip install platformio
cd firmware
pio test -e native
```

### Browser simulation (Wokwi)

1. Go to [wokwi.com](https://wokwi.com/projects/new/esp32c3)
2. Copy `wokwi/sketch.ino` into the code editor
3. Copy `wokwi/diagram.json` into the diagram editor
4. Press play — see the OLED display, turn the pots, press anchor

### Flash to hardware

```bash
# Pod 2 (motor brain — ESP32)
pio run -e esp32 -t upload

# Pod 1 (controller — ESP32-C3)
pio run -e esp32c3 -t upload
```

## License

GPL-3.0-or-later. See [LICENSE](../LICENSE) in repo root.
