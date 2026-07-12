#pragma once
#include <stdint.h>
#include <string.h>

// ============== PACKED PACKET STRUCTS ==============
// CRITICAL: Must be packed for wire transmission (Round 2 fix R2-C2)

#pragma pack(push, 1)

// Control Packet: Pod 1 → Pod 2 (50-100Hz)
struct ControlPacket {
    uint8_t  device_id;
    uint8_t  fw_version;
    uint16_t seq_num;
    uint16_t speed_raw;      // 0-4095 (ADC)
    uint16_t steering_raw;   // 0-4095 (ADC)
    uint8_t  direction;      // 0=fwd, 1=off, 2=rev
    uint8_t  anchor_btn;     // 0=released, 1=pressed
    uint16_t heading_raw;    // 0-359 degrees
    uint8_t  ctrl_batt_pct;  // 0-100
    uint8_t  flags;
    uint16_t checksum;       // CRC16
};
// Expected: 16 bytes

// Telemetry Packet: Pod 2 → Pod 1 (10Hz)
struct TelemetryPacket {
    uint8_t  device_id;
    uint8_t  fw_version;
    uint16_t seq_num;
    float    lat;
    float    lon;
    float    sog;          // m/s
    float    cog;          // degrees
    uint8_t  satellites;
    float    hdop;
    float    main_voltage;
    uint8_t  motor_mode;   // 0=manual, 1=spot-lock, 2=error
    int16_t  steer_angle;  // degrees × 10
    float    anchor_dist;  // meters
    uint16_t anchor_brg;   // 0-359 degrees
    uint8_t  error_code;
    int8_t   rssi;
    uint32_t uptime_ms;
    uint16_t checksum;
    uint16_t reserved;   // 48 — pad to even boundary
};
// Expected: 48 bytes packed
// Expected: 50 bytes

#pragma pack(pop)

// ============== CONSTANTS ==============

#define DIR_FORWARD  0
#define DIR_OFF      1
#define DIR_REVERSE  2

#define MODE_MANUAL     0
#define MODE_SPOT_LOCK  1
#define MODE_ERROR      2

// Error codes
#define ERR_OK              0
#define ERR_RF_LOST         1
#define ERR_GPS_NO_FIX      2
#define ERR_COMPASS         3
#define ERR_LOW_BATTERY     4
#define ERR_CRIT_BATTERY    5
#define ERR_ESC_OVERTEMP    6
#define ERR_WATCHDOG        7
#define ERR_SHOOT_THROUGH   8
#define ERR_GPS_STALE       9

// ============== CRC16 ==============

inline uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

inline void set_checksum(ControlPacket& pkt) {
    // Checksum covers everything except the checksum field itself
    pkt.checksum = 0;
    pkt.checksum = crc16((uint8_t*)&pkt, sizeof(pkt) - 2);
}

inline bool verify_checksum(const ControlPacket& pkt) {
    uint16_t expected = crc16((uint8_t*)&pkt, sizeof(pkt) - 2);
    return pkt.checksum == expected;
}

inline void set_checksum(TelemetryPacket& pkt) {
    pkt.checksum = 0;
    pkt.checksum = crc16((uint8_t*)&pkt, sizeof(pkt) - 4); // minus checksum + reserved
}

inline bool verify_checksum(const TelemetryPacket& pkt) {
    uint16_t expected = crc16((uint8_t*)&pkt, sizeof(pkt) - 4);
    return pkt.checksum == expected;
}
