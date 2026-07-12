#pragma once
#include <math.h>

// ============== NAVIGATION MATH ==============
// All using float (not double) for ESP32 performance (Round 2 fix R2-I5)

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

inline float radians(float deg) { return deg * M_PI / 180.0f; }
inline float degrees(float rad) { return rad * 180.0f / M_PI; }

// Normalize heading to [0, 360)
inline float normalize_heading(float h) {
    while (h < 0.0f) h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

// CRITICAL (Round 2 fix R2-C5): Heading wraparound-safe error
// Returns error in [-180, +180]
inline float heading_error(float target, float current) {
    float e = target - current;
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

// Local tangent plane position (replaces haversine for speed)
// Convert GPS delta to local meters at a reference point
struct LocalPos {
    float east_m;
    float north_m;
};

inline LocalPos to_local(float ref_lat, float ref_lon, float lat, float lon) {
    LocalPos p;
    float dlat = radians(lat - ref_lat);
    float dlon = radians(lon - ref_lon);
    float cos_lat = cosf(radians(ref_lat));
    p.north_m = dlat * 6371000.0f;
    p.east_m  = dlon * 6371000.0f * cos_lat;
    return p;
}

inline float local_distance(LocalPos p) {
    return sqrtf(p.east_m * p.east_m + p.north_m * p.north_m);
}

inline float local_bearing(LocalPos p) {
    return normalize_heading(degrees(atan2f(p.east_m, p.north_m)));
}

// Full haversine (for verification/testing only)
inline float haversine(float lat1, float lon1, float lat2, float lon2) {
    float R = 6371000.0f;
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    float a = sinf(dLat/2) * sinf(dLat/2) +
              cosf(radians(lat1)) * cosf(radians(lat2)) *
              sinf(dLon/2) * sinf(dLon/2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    return R * c;
}
