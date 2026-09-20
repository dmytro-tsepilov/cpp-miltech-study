#pragma once

#include <cstdint>
#include <string>

// Data structures for reporting (mirrors ROS messages)
struct PerimeterStatus {
    uint8_t mode = 0;
    uint32_t waypoint_index = 0;
    double target_x = 0.0;
    double target_y = 0.0;
    double current_x = 0.0;
    double current_y = 0.0;
    double lateral_error = 0.0;
    float speed = 0.0f;
    bool mine_detected = false;
    std::string timestamp;
};

struct MineDetection {
    int32_t mine_id = 0;
    double x = 0.0;
    double y = 0.0;
    std::string type;
    float confidence = 0.0f;
    std::string detected_at;
};

struct ClearanceReport {
    int32_t mine_id = 0;
    double x = 0.0;
    double y = 0.0;
    std::string method;
    bool success = false;
    std::string details;
    std::string timestamp;
};

struct MissionSummary {
    std::string scenario_name;
    std::string result;
    std::string reason;
    uint32_t total_waypoints = 0;
    uint32_t waypoints_completed = 0;
    uint32_t mines_detected = 0;
    uint32_t mines_cleared = 0;
    double mission_duration = 0.0;
    double coverage_percent = 0.0;
    std::string start_time;
    std::string end_time;
};

/// HTTP reporter for sending mission data to remote API
class HttpReporter {
public:
    /// Create reporter with API endpoint
    explicit HttpReporter(
        const std::string& endpoint = "http://localhost:8080",
        int timeout_ms = 5000);
    
    ~HttpReporter();
    
    // Send periodic movement report
    bool sendMovementReport(const PerimeterStatus& status);
    
    // Send mine detection event
    bool sendMineDetection(const MineDetection& detection);
    
    // Send clearance report event
    bool sendClearanceReport(const ClearanceReport& report);
    
    // Send mission summary (end of mission)
    bool sendMissionSummary(const MissionSummary& summary);
    
    // Get last error message
    const std::string& getLastError() const { return last_error_; }
    
    // Get success count
    int getSuccessCount() const { return success_count_; }
    
    // Get failure count
    int getFailureCount() const { return failure_count_; }

private:
    std::string endpoint_;
    int timeout_ms_;
    std::string last_error_;
    int success_count_ = 0;
    int failure_count_ = 0;
    
    // HTTP POST request via curl (private)
    bool postJson(const std::string& path, const std::string& json_data);
    
public:
    // Serialization methods (public for testing)
    std::string serializeMovementReport(const PerimeterStatus& status) {
        return serializeMovementReportImpl(status);
    }
    std::string serializeMineDetection(const MineDetection& detection) {
        return serializeMineDetectionImpl(detection);
    }
    std::string serializeClearanceReport(const ClearanceReport& report) {
        return serializeClearanceReportImpl(report);
    }
    std::string serializeMissionSummary(const MissionSummary& summary) {
        return serializeMissionSummaryImpl(summary);
    }
    std::string buildUrl(const std::string& path) const {
        return buildUrlImpl(path);
    }

private:
    // Actual implementation methods
    std::string serializeMovementReportImpl(const PerimeterStatus& status);
    std::string serializeMineDetectionImpl(const MineDetection& detection);
    std::string serializeClearanceReportImpl(const ClearanceReport& report);
    std::string serializeMissionSummaryImpl(const MissionSummary& summary);
    std::string buildUrlImpl(const std::string& path) const;
};
