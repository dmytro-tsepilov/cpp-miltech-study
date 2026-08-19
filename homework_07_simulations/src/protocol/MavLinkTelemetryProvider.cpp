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

bool MavLinkTelemetryProvider::sendMavlinkMessage(const mavlink_message_t& message) {
    if (sockfd_ < 0) return false;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t totalMsgLen = mavlink_msg_to_send_buffer(buf, &message);

    ssize_t sent = sendto(sockfd_, buf, totalMsgLen, 0,
                          reinterpret_cast<sockaddr*>(&destAddr_), sizeof(destAddr_));
    if (sent < 0) {
        std::cerr << "[MavLink] sendto failed for msgid=" << message.msgid
                  << " target=" << targetIp_ << ":" << targetPort_
                  << ": " << strerror(errno) << std::endl;
        return false;
    }
    if (sent != totalMsgLen) {
        std::cerr << "[MavLink] short UDP send for msgid=" << message.msgid
                  << ": " << sent << "/" << totalMsgLen << " bytes" << std::endl;
        return false;
    }
    return true;
}

void MavLinkTelemetryProvider::packHeartbeat(mavlink_message_t& message) {
    mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &message,
                                MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                                0, 0, MAV_STATE_ACTIVE);
}

void MavLinkTelemetryProvider::packGlobalPositionInt(mavlink_message_t& message) {
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

    int32_t alt_mm = static_cast<int32_t>(lastAltitude_ * 1000.0f);
    int32_t relAlt_mm = static_cast<int32_t>(lastRelativeAlt_ * 1000.0f);
    int16_t vx_cm = static_cast<int16_t>(lastVx_ * 100.0f);
    int16_t vy_cm = static_cast<int16_t>(lastVy_ * 100.0f);
    uint16_t hdg_cdeg = static_cast<uint16_t>(lastHeadingDeg_ * 100.0f);
    mavlink_msg_global_position_int_pack(SYS_ID, COMP_ID, &message,
                                         timeBootMs, lat, lon, alt_mm,
                                         relAlt_mm, vx_cm, vy_cm, 0, hdg_cdeg);
}

void MavLinkTelemetryProvider::packAttitude(mavlink_message_t& message) {
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

    mavlink_msg_attitude_pack(SYS_ID, COMP_ID, &message,
                              timeBootMs, 0.0f, 0.0f, headingRad,
                              0.0f, 0.0f, 0.0f);
}

void MavLinkTelemetryProvider::packCommandLong(mavlink_message_t& message) {
    double dropLatDeg, dropLonDeg;
    {
        std::lock_guard<std::mutex> lock(telMutex_);
        int32_t latInt, lonInt;
        localToGps(dropLocalX_, dropLocalY_, latInt, lonInt);
        dropLatDeg = latInt / 1e7;
        dropLonDeg = lonInt / 1e7;
    }

    float dropAlt = dropAltitude_;

    float params[7] = {0.0f, 0.0f, 0.0f, 0.0f,
                       static_cast<float>(dropLatDeg),
                       static_cast<float>(dropLonDeg),
                       dropAlt};
    mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &message,
                                  1, MAV_COMP_ID_AUTOPILOT1, MAV_CMD_USER_1,
                                  1, params[0], params[1], params[2], params[3],
                                  params[4], params[5], params[6]);
}

void MavLinkTelemetryProvider::sendTelemetry(double localX, double localY, float altitude,
                                               float relativeAlt, float vx, float vy,
                                               float headingDeg, uint32_t timeBootMs) {
    if (!running_.load()) {
        return;
    }

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

    // Send GLOBAL_POSITION_INT
    mavlink_message_t globalPositionMessage{};
    packGlobalPositionInt(globalPositionMessage);
    int gpSent = sendMavlinkMessage(globalPositionMessage);

    // Send ATTITUDE (24 bytes payload)
    mavlink_message_t attitudeMessage{};
    packAttitude(attitudeMessage);
    int attSent = sendMavlinkMessage(attitudeMessage);

    // Log first few telemetry frames for debugging
    static int telLogCount = 0;
    if (telLogCount < 5) {
        std::cout << "[MavLink] telemetry call: gp=" << gpSent << " att=" << attSent
                  << " pos=(" << localX << "," << localY << ")"
                  << " time=" << timeBootMs << "ms" << std::endl;
        telLogCount++;
    }
}

bool MavLinkTelemetryProvider::sendDropCommand(double localX, double localY, float altitude) {
    std::unique_lock<std::mutex> lock(dropMutex_);

    dropLocalX_ = localX;
    dropLocalY_ = localY;
    dropAltitude_ = altitude;
    dropRetryCount_ = 0;
    dropCmdPending_ = true;

    // Compute actual GPS coordinates for logging
    int32_t latInt, lonInt;
    localToGps(dropLocalX_, dropLocalY_, latInt, lonInt);
    double dropLatDeg = latInt / 1e7;
    double dropLonDeg = lonInt / 1e7;

    std::cout << "[MavLink] Sending DROP command (lat=" << dropLatDeg
              << ", lon=" << dropLonDeg << "), altitude=" << altitude << "m" << std::endl;

    const int maxRetries = 5;
    const std::chrono::milliseconds retryDelay(500);

    for (int i = 0; i < maxRetries; ++i) {
        dropRetryCount_ = i + 1;

        // Send COMMAND_LONG
        mavlink_message_t commandMessage{};
        packCommandLong(commandMessage);
        sendMavlinkMessage(commandMessage);

        std::cout << "[MavLink] DROP attempt " << dropRetryCount_ << "/" << maxRetries << std::endl;

        // The receiver thread owns recvfrom(). Wait until it processes the ACK.
        const bool ackReceived = dropCv_.wait_for(
            lock, std::chrono::seconds(2),
            [this] { return !dropCmdPending_.load(); });

        if (ackReceived || !dropCmdPending_.load()) {
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

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    struct sockaddr_in srcAddr;
    socklen_t srcLen = sizeof(srcAddr);

    ssize_t n = recvfrom(sockfd_, buf, sizeof(buf), 0,
                         reinterpret_cast<sockaddr*>(&srcAddr), &srcLen);
    if (n <= 0) return;

    mavlink_message_t message{};
    for (ssize_t i = 0; i < n; ++i) {
        if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &message, &mavlinkStatus_)) {
            continue;
        }

        if (message.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
            continue;
        }

        const uint16_t command = mavlink_msg_command_ack_get_command(&message);
        const uint8_t result = mavlink_msg_command_ack_get_result(&message);
        if (command != MAV_CMD_USER_1) {
            continue;
        }

        if (result == MAV_RESULT_ACCEPTED) {
            std::cout << "[MavLink] COMMAND_ACK received for MAV_CMD_USER_1" << std::endl;
        } else {
            std::cerr << "[MavLink] COMMAND_ACK with result=" << static_cast<int>(result)
                      << " for MAV_CMD_USER_1" << std::endl;
        }
        dropCmdPending_ = false;
        dropCv_.notify_all();
    }
}

void MavLinkTelemetryProvider::receiveLoop() {
    while (running_.load()) {
        parseIncoming();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void MavLinkTelemetryProvider::heartbeatLoop() {
    mavlink_message_t heartbeatMessage{};
    packHeartbeat(heartbeatMessage);
    while (running_.load()) {
        const bool sent = sendMavlinkMessage(heartbeatMessage);
        std::cout << "[MavLink] heartbeat: " << (sent ? "sent" : "failed")
                  << " -> " << targetIp_ << ":" << targetPort_ << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
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
    heartbeatThread_ = std::thread(&MavLinkTelemetryProvider::heartbeatLoop, this);

    std::cout << "[MavLink] Telemetry provider started -> " << targetIp_ << ":" << targetPort_ << std::endl;
    return true;
}

void MavLinkTelemetryProvider::stop() {
    if (!running_.load()) return;

    running_ = false;

    if (recvThread_.joinable()) {
        recvThread_.join();
    }
    if (heartbeatThread_.joinable()) {
        heartbeatThread_.join();
    }

    closeUdpSocket();
    std::cout << "[MavLink] Telemetry provider stopped" << std::endl;
}
