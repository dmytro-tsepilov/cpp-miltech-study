#pragma once
// MavLinkTelemetryProvider — sends MAVLink 2 telemetry over UDP for UART mode.
//
// Sends:
//   * HEARTBEAT at ~1 Hz (MAV_TYPE_QUADROTOR, MAV_STATE_ACTIVE)
//   * GLOBAL_POSITION_INT at >= 2 Hz (lat/lon from local coords, alt in mm, velocity in cm/s)
//   * ATTITUDE at >= 2 Hz (yaw from heading, roll/pitch = 0)
//   * COMMAND_LONG on payload drop with retry up to 5 attempts
//
// Listens for:
//   * COMMAND_ACK matching the sent drop command
//
// Reference coordinates for local-to-GPS conversion:
//   lat0 = 50.4501, lon0 = 30.5234

#ifndef MAVLINK_TELEMETRY_PROVIDER_H
#define MAVLINK_TELEMETRY_PROVIDER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <cstdint>

#include <common/mavlink.h>

class MavLinkTelemetryProvider {
public:
    // Default target: 127.0.0.1:14550 (QGroundControl listens on this by default)
    static constexpr const char* DEFAULT_TARGET_IP = "10.0.10.17";
    static constexpr int DEFAULT_TARGET_PORT = 14550;

    // Reference coordinates for local-to-GPS conversion
    static constexpr double REF_LAT = 50.4501;
    static constexpr double REF_LON = 30.5234;

    MavLinkTelemetryProvider();
    ~MavLinkTelemetryProvider();

    // Initialize UDP socket and start telemetry thread
    // Returns true on success, false on failure
    bool init(const std::string& targetIp = DEFAULT_TARGET_IP,
              int targetPort = DEFAULT_TARGET_PORT);

    // Stop and cleanup
    void stop();

    // ---- Telemetry update API (thread-safe) ----
    // Call from physics loop at each step
    void sendTelemetry(double localX, double localY, float altitude,
                       float relativeAlt, float vx, float vy,
                       float headingDeg, uint32_t timeBootMs);

    // ---- Drop command API ----
    // Called when mission decides to drop payload
    // Converts local coords to GPS and sends COMMAND_LONG
    // Returns true if ACK received, false after 5 retries
    bool sendDropCommand(double localX, double localY, float altitude);

    // Check if a drop command is currently pending (no ACK yet)
    bool isDropPending() const { return dropCmdPending_.load(); }

    // Get current status
    bool isRunning() const { return running_.load(); }

private:
    // Convert local coordinates to GPS
    void localToGps(double localX, double localY,
                    int32_t& lat, int32_t& lon) const;

    // UDP socket helpers
    bool initUdpSocket();
    void closeUdpSocket();
    bool sendMavlinkMessage(const mavlink_message_t& message);

    // MAVLink message packers (manual packing for c_library_v2)
    void packHeartbeat(mavlink_message_t& message);
    void packGlobalPositionInt(mavlink_message_t& message);
    void packAttitude(mavlink_message_t& message);
    void packCommandLong(mavlink_message_t& message);

    // MAVLink message unpacker (for COMMAND_ACK)
    void parseIncoming();

    // Background thread for receiving COMMAND_ACK
    void receiveLoop();

    // Heartbeat timer
    void heartbeatLoop();

    // State
    int sockfd_ = -1;
    struct sockaddr_in destAddr_;
    std::string targetIp_;
    int targetPort_;

    std::atomic<bool> running_{false};
    std::thread recvThread_;
    std::thread heartbeatThread_;
    mavlink_status_t mavlinkStatus_{};

    // Drop command state
    mutable std::mutex dropMutex_;
    double dropLocalX_ = 0.0;
    double dropLocalY_ = 0.0;
    float dropAltitude_ = 0.0;
    int dropRetryCount_ = 0;
    std::atomic<bool> dropCmdPending_{false};
    std::condition_variable dropCv_;

    // Last telemetry data (updated by sendTelemetry)
    mutable std::mutex telMutex_;
    double lastLocalX_ = 0.0;
    double lastLocalY_ = 0.0;
    float lastAltitude_ = 0.0;
    float lastRelativeAlt_ = 0.0;
    float lastVx_ = 0.0f;
    float lastVy_ = 0.0f;
    float lastHeadingDeg_ = 0.0f;
    uint32_t lastTimeBootMs_ = 0;

    // Heartbeat timer
    uint32_t lastHeartbeatTimeMs_ = 0;

    // MAVLink system/component IDs
    static constexpr uint8_t SYS_ID = 1;
    static constexpr uint8_t COMP_ID = 4;  // MAV_COMP_ID_AUTOPILOT1

    // Drop command tracking
    static constexpr uint16_t DROP_COMMAND = 50000;  // MAV_CMD_USER_1
};

#endif // MAVLINK_TELEMETRY_PROVIDER_H
