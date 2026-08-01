#pragma once
// drone_link.h — binary UART protocol for HW22 (Lesson 4.4)
// Shared between physics simulator, checker and student program.
//
// Frame on wire:
//   [0]  MAGIC0 = 0xA5
//   [1]  MAGIC1 = 0x5A
//   [2]  TYPE   (1 byte)        — packet type (PacketType)
//   [3]  LEN    (1 byte)        — payload length in bytes
//   [4..4+LEN-1]  payload       — raw data (little-endian)
//   [..] CRC16  (2 bytes, LE)   — CRC-16/CCITT-FALSE over TYPE+LEN+payload
//
// All multi-byte fields are little-endian (as on ARM/x86 by default).
// Frame is self-synchronizing: receiver searches for MAGIC0,MAGIC1, then reads LEN and CRC.

#ifndef DRONE_LINK_H
#define DRONE_LINK_H

#include <cstdint>
#include <cstring>

namespace dlink {

constexpr uint8_t MAGIC0 = 0xA5;
constexpr uint8_t MAGIC1 = 0x5A;

enum PacketType : uint8_t {
    PKT_TELEMETRY = 0x01,  // drone state at a point in time
    PKT_TARGET    = 0x02,  // position of one target (sent periodically per target)
    PKT_AMMO      = 0x03,  // ammo parameters + hitRadius (once at start)
    PKT_RESULT    = 0x04,  // checker verdict (HIT/MISS) — only on real Pi back over UART
    PKT_CONTROL   = 0x05,  // CONTROL: student -> checker (acceleration and turn rate)
    PKT_CONFIG    = 0x06,  // mission parameters from config (checker sends to student at start)
};

#pragma pack(push, 1)

// PKT_TELEMETRY — what the student reads and parses
struct Telemetry {
    uint32_t t_ms;     // time since start, milliseconds (timestamp)
    float    x, y;     // drone position in plane, meters
    float    z;        // altitude, meters
    float    vx, vy;   // velocity in plane, m/s
    float    speed;    // horizontal speed modulus, m/s
    float    dir;      // heading (flight direction), radians
    uint8_t  state;    // state machine state (0..4, as in DZ3)
};

// PKT_TARGET — target position "now" (target may move)
struct TargetPos {
    uint8_t  id;       // target index
    float    x, y;     // current target position, meters
};

// PKT_AMMO — shot configuration (sent once at start)
struct AmmoCfg {
    char     name[16]; // e.g. "VOG-17"
    float    mass;     // m
    float    drag;     // d
    float    lift;     // l
    float    hitRadius;// successful hit radius, meters
    uint8_t  nTargets; // how many targets in mission
};

// PKT_RESULT — verdict (reverse channel on hardware)
struct Result {
    uint8_t  hit;      // 1 = hit, 0 = miss
    uint8_t  targetId; // which target (or 0xFF)
    float    miss_m;   // miss distance, meters
    uint32_t drop_t_ms;// when drop was triggered
};

// PKT_CONTROL — drone control command (student sends to checker every tick)
// Normalized values; checker multiplies by physical limits (maxAccel, maxTurnRate).
struct Control {
    float accel;       // acceleration along heading, [-1..1] (1 = full throttle, -1 = brake)
    float turnRate;    // turn rate, [-1..1] (1 = max left, -1 = right)
};

// PKT_CONFIG — mission parameters from config (checker sends to student once at start, like AMMO).
// These are the DZ9 config fields not present in TELEMETRY/AMMO. position/altitude/dir come
// from telemetry, hitRadius and ammo params — from AMMO.
struct DroneCfg {
    float attackSpeed;          // max drone speed, m/s
    float accelerationPath;     // acceleration path to attackSpeed, meters (accel = v^2/(2*path))
    float angularSpeed;         // max angular turn speed, rad/s
    float turnThreshold;        // turn threshold, radians
    float timeStep;             // simulation step, seconds
    float timeScale;            // simulation acceleration (1 = real time; set by checker arg)
};

#pragma pack(pop)

// ---- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) ----
inline uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// Pack a frame into buffer out (returns number of bytes written).
// out must be >= 6 + payloadLen.
inline size_t encode(uint8_t type, const void* payload, uint8_t payloadLen,
                     uint8_t* out) {
    out[0] = MAGIC0;
    out[1] = MAGIC1;
    out[2] = type;
    out[3] = payloadLen;
    if (payloadLen && payload) std::memcpy(out + 4, payload, payloadLen);
    uint16_t c = crc16(out + 2, (size_t)payloadLen + 2);  // over TYPE+LEN+payload
    out[4 + payloadLen]     = (uint8_t)(c & 0xFF);
    out[4 + payloadLen + 1] = (uint8_t)(c >> 8);
    return (size_t)payloadLen + 6;
}

// Incremental parser: feed bytes, feed() returns true when a complete valid frame is assembled.
// Maintains state between calls — convenient for reading from UART in chunks.
struct Parser {
    enum { S_M0, S_M1, S_TYPE, S_LEN, S_PAYLOAD, S_CRC0, S_CRC1 } st = S_M0;
    uint8_t  type = 0, len = 0, idx = 0;
    uint8_t  buf[260];
    uint16_t crc_rx = 0;

    // Returns true when a complete valid frame is assembled.
    // type/payload/outLen are valid only when true.
    bool feed(uint8_t byte, uint8_t& outType, uint8_t* outPayload, uint8_t& outLen) {
        switch (st) {
        case S_M0: if (byte == MAGIC0) st = S_M1; break;
        case S_M1: st = (byte == MAGIC1) ? S_TYPE : S_M0; break;
        case S_TYPE: type = byte; st = S_LEN; break;
        case S_LEN:  len = byte; idx = 0; st = len ? S_PAYLOAD : S_CRC0; break;
        case S_PAYLOAD:
            buf[idx++] = byte;
            if (idx >= len) st = S_CRC0;
            break;
        case S_CRC0: crc_rx = byte; st = S_CRC1; break;
        case S_CRC1: {
            crc_rx |= (uint16_t)byte << 8;
            st = S_M0;
            // recompute CRC over TYPE+LEN+payload
            uint8_t tmp[262];
            tmp[0] = type; tmp[1] = len;
            std::memcpy(tmp + 2, buf, len);
            if (crc16(tmp, (size_t)len + 2) == crc_rx) {
                outType = type; outLen = len;
                std::memcpy(outPayload, buf, len);
                return true;
            }
            break; // corrupted data — discard frame, sync further ahead
        }
        }
        return false;
    }
};

} // namespace dlink

#endif // DRONE_LINK_H
