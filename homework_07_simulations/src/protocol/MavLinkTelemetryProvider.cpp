#include "protocol/MavLinkTelemetryProvider.h"

#include <cstring>
#include <cmath>
#include <iostream>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// MAVLink constants
static constexpr uint8_t MAV_TYPE_QUADROTOR = 15;
static constexpr uint8_t MAV_AUTOPILOT_GENERIC = 4;
static constexpr uint8_t MAV_STATE_ACTIVE = 3;
static constexpr uint16_t MAV_CMD_USER_1 = 50000;
static constexpr uint8_t MAV_RESULT_ACCEPTED = 1;

// Message IDs in the minimal MAVLink 2 message format we use
static constexpr uint8_t MAVLINK_MSG_ID_HEARTBEAT = 0;
static constexpr uint8_t MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 62;
static constexpr uint8_t MAVLINK_MSG_ID_ATTITUDE = 30;
static constexpr uint8_t MAVLINK_MSG_ID_COMMAND_LONG = 76;
static constexpr uint8_t MAVLINK_MSG_ID_COMMAND_ACK = 187;

MavLinkTelemetryProvider::MavLinkTelemetryProvider()
    : targetIp_(DEFAULT_TARGET_IP), targetPort_(DEFAULT_TARGET_PORT) {
}

MavLinkTelemetryProvider::~MavLinkTelemetryProvider() {
    stop();
}

void MavLinkTelemetryProvider::localToGps(double localX, double localY,
                                           int32_t& lat, int32_t& lon) const {
    // lat = lat0 + (y / 111320.0)
    // lon = lon0 + (x / (111320.0 * cos(lat0 * PI/180)))
    double latDeg = REF_LAT + (localY / 111320.0);
    double lonDeg = REF_LON + (localX / (111320.0 * std::cos(REF_LAT * M_PI / 180.0)));

    // MAVLink expects degrees * 1e7 as int32_t
    lat = static_cast<int32_t>(latDeg * 1e7);
    lon = static_cast<int32_t>(lonDeg * 1e7);
}

bool MavLinkTelemetryProvider::initUdpSocket() {
    sockfd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
    if (sockfd_ < 0) {
        std::cerr << "[MavLink] Failed to create UDP socket: " << strerror(errno) << std::endl;
        return false;
    }

    // Set receive timeout for COMMAND_ACK polling
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000; // 200ms timeout
    if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        std::cerr << "[MavLink] Failed to set socket timeout: " << strerror(errno) << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }

    memset(&destAddr_, 0, sizeof(destAddr_));
    destAddr_.sin_family = AF_INET;
    destAddr_.sin_port = htons(targetPort_);

    if (inet_pton(AF_INET, targetIp_.c_str(), &destAddr_.sin_addr) <= 0) {
        std::cerr << "[MavLink] Invalid target address: " << targetIp_ << std::endl;
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }

    std::cout << "[MavLink] UDP socket opened -> " << targetIp_ << ":" << targetPort_ << std::endl;
    return true;
}

void MavLinkTelemetryProvider::closeUdpSocket() {
    if (sockfd_ >= 0) {
        close(sockfd_);
        sockfd_ = -1;
    }
}

bool MavLinkTelemetryProvider::sendMavlinkMessage(uint8_t msgId, const uint8_t* payload, uint16_t len) {
    if (sockfd_ < 0) return false;

    // Build a minimal MAVLink 2 message manually:
    // [0]    : magic (0xFD)
    // [1..2] : payload length
    // [3]    : msg ID (low byte)
    // [4]    : msg ID (mid byte)
    // [5]    : msg ID (high byte)
    // [6..6+len-1] : payload
    // [6+len]  : checksum

    uint8_t buf[256];
    if (6 + len + 1 > sizeof(buf)) return false;

    buf[0] = 0xFD;                          // MAVLink 2 magic
    buf[1] = static_cast<uint8_t>(len);     // payload length
    buf[2] = static_cast<uint8_t>(msgId & 0xFF);
    buf[3] = static_cast<uint8_t>((msgId >> 8) & 0xFF);
    buf[4] = static_cast<uint8_t>((msgId >> 16) & 0xFF);
    buf[5] = COMP_ID;                       // component ID

    std::memcpy(buf + 6, payload, len);

    // Simple checksum over header + payload
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < 6 + len; ++i) {
        checksum += buf[i];
    }
    buf[6 + len] = static_cast<uint8_t>(checksum & 0xFF);

    ssize_t sent = sendto(sockfd_, buf, 7 + len, 0,
                          reinterpret_cast<sockaddr*>(&destAddr_), sizeof(destAddr_));
    if (sent < 0) {
        // Don't log errors for heartbeat - it's frequent
        return false;
    }
    return true;
}

void MavLinkTelemetryProvider::packHeartbeat(uint8_t* payload, uint16_t& len) {
    // HEARTBEAT message format (7 bytes):
    // type (1), mavType (1), baseMode (1), customMode (4) - but we only need first bytes
    // Actually for MAVLink 2 with COMP_ID:
    // We pack into the payload area after header

    // Minimal heartbeat: just type and status
    payload[0] = static_cast<uint8_t>(MAV_TYPE_QUADROTOR);       // type
    payload[1] = static_cast<uint8_t>(MAV_AUTOPILOT_GENERIC);    // mavType
    payload[2] = 0;                                                // baseMode (logical OR of MAV_MODE_FLAG)
    payload[3] = 0;                                                // customMode reserved
    payload[4] = 0;
    payload[5] = 0;
    payload[6] = static_cast<uint8_t>(MAV_STATE_ACTIVE);         // systemStatus
    len = 7;
}

void MavLinkTelemetryProvider::packGlobalPositionInt(uint8_t* payload, uint16_t& len) {
    int32_t lat = 0, lon = 0;
    {
        std::lock_guard<std::mutex> lock(telMutex_);
        localToGps(lastLocalX_, lastLocalY_, lat, lon);
    }

    // GLOBAL_POSITION_INT (30 bytes):
    // time_boot_ms (u32), lat (i32), lon (i32), alt (i32), relative_alt (i32),
    // vx (i16), vy (i16), vz (i16), hdg (u16)
    uint32_t timeBootMs;
    {
        std::lock_guard<std::mutex> lock(telMutex_);
        timeBootMs = lastTimeBootMs_;
    }

    // Pack in little-endian order
    uint8_t* p = payload;
    std::memcpy(p, &timeBootMs, 4); p += 4;
    int32_t latVal = lat;
    std::memcpy(p, &latVal, 4); p += 4;
    int32_t lonVal = lon;
    std::memcpy(p, &lonVal, 4); p += 4;

    int32_t alt_mm = static_cast<int32_t>(lastAltitude_ * 1000.0f);
    std::memcpy(p, &alt_mm, 4); p += 4;

    int32_t relAlt_mm = static_cast<int32_t>(lastRelativeAlt_ * 1000.0f);
    std::memcpy(p, &relAlt_mm, 4); p += 4;

    int16_t vx_cm = static_cast<int16_t>(lastVx_ * 100.0f);
    std::memcpy(p, &vx_cm, 2); p += 2;

    int16_t vy_cm = static_cast<int16_t>(lastVy_ * 100.0f);
    std::memcpy(p, &vy_cm, 2); p += 2;

    int16_t vz_cm = 0; // We don't have vertical velocity
    std::memcpy(p, &vz_cm, 2); p += 2;

    uint16_t hdg_cdeg = static_cast<uint16_t>(lastHeadingDeg_ * 100.0f);
    std::memcpy(p, &hdg_cdeg, 2); p += 2;

    len = 30;
}

void MavLinkTelemetryProvider::packAttitude(uint8_t* payload, uint16_t& len) {
    uint32_t timeBootMs;
    {
        std::lock_guard<std::mutex> lock(telMutex_);
        timeBootMs = lastTimeBootMs_;
    }

    float headingRad;
    {
        std::lock_guard<std::mutex> lock(telMutex_);
        // Convert degrees to radians for ATTITUDE message
        headingRad = lastHeadingDeg_ * M_PI / 180.0f;
    }

    // ATTITUDE (24 bytes):
    // time_boot_ms (u32), roll (f), pitch (f), yaw (f)
    uint8_t* p = payload;
    std::memcpy(p, &timeBootMs, 4); p += 4;

    float roll = 0.0f;
    std::memcpy(p, &roll, 4); p += 4;

    float pitch = 0.0f;
    std::memcpy(p, &pitch, 4); p += 4;

    std::memcpy(p, &headingRad, 4); p += 4;

    // Zero padding for remaining fields if needed
    len = 16;
}

void MavLinkTelemetryProvider::packCommandLong(uint8_t* payload, uint16_t& len) {
    double dropLatDeg, dropLonDeg;
    {
        std::lock_guard<std::mutex> lock(telMutex_);
        int32_t latInt, lonInt;
        localToGps(dropLocalX_, dropLocalY_, latInt, lonInt);
        dropLatDeg = latInt / 1e7;
        dropLonDeg = lonInt / 1e7;
    }

    float dropAlt = dropAltitude_;

    // COMMAND_LONG (58 bytes):
    // target_system (u8), target_component (u8), command (u16),
    // param1-7 (f each), confirmation (u8)
    uint8_t* p = payload;
    p[0] = SYS_ID;        // target_system
    p[1] = COMP_ID;       // target_component
    p += 2;

    uint16_t cmd = MAV_CMD_USER_1;
    std::memcpy(p, &cmd, 2); p += 2;

    float params[7] = {0.0f, 0.0f, 0.0f, 0.0f,
                       static_cast<float>(dropLatDeg),
                       static_cast<float>(dropLonDeg),
                       dropAlt};
    std::memcpy(p, params, sizeof(params)); p += sizeof(params);

    p[0] = 1; // confirmation
    len = 58;
}

void MavLinkTelemetryProvider::sendTelemetry(double localX, double localY, float altitude,
                                              float relativeAlt, float vx, float vy,
                                              float headingDeg, uint32_t timeBootMs) {
    if (!running_.load()) return;

    // Update last telemetry data
    {
        std::lock_guard<std::mutex> lock(telMutex_);
        lastLocalX_ = localX;
        lastLocalY_ = localY;
        lastAltitude_ = altitude;
        lastRelativeAlt_ = relativeAlt;
        lastVx_ = vx;
        lastVy_ = vy;
        lastHeadingDeg_ = headingDeg;
        lastTimeBootMs_ = timeBootMs;
    }

    // Send HEARTBEAT (throttle to ~1 Hz)
    auto now = std::chrono::steady_clock::now();
    uint32_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    if (nowMs - lastHeartbeatTimeMs_ > 1000) {
        uint8_t hbPayload[7];
        uint16_t hbLen = 0;
        packHeartbeat(hbPayload, hbLen);
        sendMavlinkMessage(MAVLINK_MSG_ID_HEARTBEAT, hbPayload, hbLen);
        lastHeartbeatTimeMs_ = nowMs;
    }

    // Send GLOBAL_POSITION_INT
    uint8_t gpPayload[30];
    uint16_t gpLen = 0;
    packGlobalPositionInt(gpPayload, gpLen);
    sendMavlinkMessage(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, gpPayload, gpLen);

    // Send ATTITUDE
    uint8_t attPayload[16];
    uint16_t attLen = 0;
    packAttitude(attPayload, attLen);
    sendMavlinkMessage(MAVLINK_MSG_ID_ATTITUDE, attPayload, attLen);
}

bool MavLinkTelemetryProvider::sendDropCommand(double localX, double localY, float altitude) {
    std::lock_guard<std::mutex> lock(dropMutex_);

    dropLocalX_ = localX;
    dropLocalY_ = localY;
    dropAltitude_ = altitude;
    dropRetryCount_ = 0;
    dropCmdPending_ = true;

    std::cout << "[MavLink] Sending DROP command (lat=" << REF_LAT
              << ", lon=" << REF_LON << "), altitude=" << altitude << "m" << std::endl;

    const int maxRetries = 5;
    const std::chrono::milliseconds retryDelay(500);

    for (int i = 0; i < maxRetries; ++i) {
        dropRetryCount_ = i + 1;

        // Send COMMAND_LONG
        uint8_t cmdPayload[58];
        uint16_t cmdLen = 0;
        packCommandLong(cmdPayload, cmdLen);
        sendMavlinkMessage(MAVLINK_MSG_ID_COMMAND_LONG, cmdPayload, cmdLen);

        std::cout << "[MavLink] DROP attempt " << dropRetryCount_ << "/" << maxRetries << std::endl;

        // Wait for ACK with timeout
        auto startTime = std::chrono::steady_clock::now();
        bool ackReceived = false;
        while (std::chrono::steady_clock::now() - startTime < std::chrono::seconds(2)) {
            if (dropCmdPending_.load()) {
                // Check incoming messages
                parseIncoming();
                if (!dropCmdPending_.load()) {
                    ackReceived = true;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (ackReceived) {
            std::cout << "[MavLink] DROP ACK received!" << std::endl;
            return true;
        }

        std::cout << "[MavLink] No ACK for attempt " << dropRetryCount_ << ", retrying..." << std::endl;
        std::this_thread::sleep_for(retryDelay);
    }

    dropCmdPending_ = false;
    std::cerr << "[MavLink] DROP failed: no ACK after " << maxRetries << " attempts" << std::endl;
    return false;
}

void MavLinkTelemetryProvider::parseIncoming() {
    if (sockfd_ < 0) return;

    uint8_t buf[256];
    struct sockaddr_in srcAddr;
    socklen_t srcLen = sizeof(srcAddr);

    ssize_t n = recvfrom(sockfd_, buf, sizeof(buf), 0,
                         reinterpret_cast<sockaddr*>(&srcAddr), &srcLen);
    if (n < 7) return; // Too short for any MAVLink message

    // Check for MAVLink 2 magic
    if (buf[0] != 0xFD) return;

    uint16_t msgId = buf[2] | (buf[3] << 8) | (buf[4] << 16);

    // Verify checksum
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < 6 + buf[1]; ++i) {
        checksum += buf[i];
    }
    if (static_cast<uint8_t>(checksum & 0xFF) != buf[6 + buf[1]]) return;

    // Process COMMAND_ACK
    if (msgId == MAVLINK_MSG_ID_COMMAND_ACK) {
        if (n >= 7 + 4) { // Need at least header + command + result
            uint16_t cmd = buf[6] | (buf[7] << 8);
            uint8_t result = buf[8];

            if (cmd == MAV_CMD_USER_1 && result == MAV_RESULT_ACCEPTED) {
                std::cout << "[MavLink] COMMAND_ACK received for MAV_CMD_USER_1" << std::endl;
                dropCmdPending_ = false;
                dropCv_.notify_all();
            } else if (cmd == MAV_CMD_USER_1) {
                std::cerr << "[MavLink] COMMAND_ACK with result=" << static_cast<int>(result)
                          << " for MAV_CMD_USER_1" << std::endl;
                dropCmdPending_ = false;
                dropCv_.notify_all();
            }
        }
    }
}

void MavLinkTelemetryProvider::receiveLoop() {
    while (running_.load()) {
        parseIncoming();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void MavLinkTelemetryProvider::heartbeatLoop() {
    // Heartbeat is handled inline in sendTelemetry for simplicity
    // This function is kept for potential future use
}

bool MavLinkTelemetryProvider::init(const std::string& targetIp, int targetPort) {
    if (running_.load()) return true;

    targetIp_ = targetIp;
    targetPort_ = targetPort;

    if (!initUdpSocket()) {
        return false;
    }

    running_ = true;
    lastHeartbeatTimeMs_ = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    recvThread_ = std::thread(&MavLinkTelemetryProvider::receiveLoop, this);

    std::cout << "[MavLink] Telemetry provider started -> " << targetIp_ << ":" << targetPort_ << std::endl;
    return true;
}

void MavLinkTelemetryProvider::stop() {
    if (!running_.load()) return;

    running_ = false;

    if (recvThread_.joinable()) {
        recvThread_.join();
    }

    closeUdpSocket();
    std::cout << "[MavLink] Telemetry provider stopped" << std::endl;
}
