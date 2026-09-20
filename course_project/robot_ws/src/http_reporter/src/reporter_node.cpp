#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "perimeter_msgs/msg/perimeter_status.hpp"
#include "perimeter_msgs/msg/mine_detection.hpp"
#include "perimeter_msgs/msg/clearance_report.hpp"
#include "perimeter_msgs/msg/mission_summary.hpp"

#include "http_reporter/http_reporter.hpp"

using namespace std::chrono_literals;

/// HTTP Reporter node - sends mission data to remote API
class ReporterNode : public rclcpp::Node {
public:
    ReporterNode()
        : Node("reporter_node")
        , api_endpoint_(declare_parameter("api_endpoint", "http://localhost:8080"))
        , reporter_(api_endpoint_, declare_parameter("timeout_ms", 5000))
    {
        RCLCPP_INFO(get_logger(), "HTTP Reporter starting...");
        RCLCPP_INFO(get_logger(), "API endpoint: %s", api_endpoint_.c_str());
        
        // Subscriptions
        movement_sub_ = create_subscription<perimeter_msgs::msg::PerimeterStatus>(
            "/perimeter/status", 10,
            [this](const perimeter_msgs::msg::PerimeterStatus::SharedPtr msg) {
                onMovementReport(*msg);
            });
        
        mine_sub_ = create_subscription<perimeter_msgs::msg::MineDetection>(
            "/mines/detected", 10,
            [this](const perimeter_msgs::msg::MineDetection::SharedPtr msg) {
                onMineDetection(*msg);
            });
        
        clearance_sub_ = create_subscription<perimeter_msgs::msg::ClearanceReport>(
            "/mines/cleared", 10,
            [this](const perimeter_msgs::msg::ClearanceReport::SharedPtr msg) {
                onClearanceReport(*msg);
            });
        
        summary_sub_ = create_subscription<perimeter_msgs::msg::MissionSummary>(
            "/mission/summary", 10,
            [this](const perimeter_msgs::msg::MissionSummary::SharedPtr msg) {
                onMissionSummary(*msg);
            });
        
        // Status publisher
        report_status_pub_ = create_publisher<std_msgs::msg::String>(
            "/reporter/status", 10);
        
        RCLCPP_INFO(get_logger(), "HTTP Reporter initialized successfully");
    }

private:
    void onMovementReport(const perimeter_msgs::msg::PerimeterStatus& msg) {
        PerimeterStatus status;
        status.mode = msg.mode;
        status.waypoint_index = msg.waypoint_index;
        status.target_x = msg.target_x;
        status.target_y = msg.target_y;
        status.current_x = msg.current_x;
        status.current_y = msg.current_y;
        status.lateral_error = msg.lateral_error;
        status.speed = msg.speed;
        status.mine_detected = msg.mine_detected;
        status.timestamp = std::to_string(msg.timestamp.sec) + "." + std::to_string(msg.timestamp.nanosec);
        
        if (reporter_.sendMovementReport(status)) {
            RCLCPP_DEBUG(get_logger(), "Movement report sent successfully");
        } else {
            RCLCPP_WARN(get_logger(), "Failed to send movement report: %s",
                        reporter_.getLastError().c_str());
        }
    }
    
    void onMineDetection(const perimeter_msgs::msg::MineDetection& msg) {
        MineDetection detection;
        detection.mine_id = msg.mine_id;
        detection.x = msg.x;
        detection.y = msg.y;
        detection.type = msg.type;
        detection.confidence = msg.confidence;
        detection.detected_at = std::to_string(msg.detected_at.sec) + "." + std::to_string(msg.detected_at.nanosec);
        
        if (reporter_.sendMineDetection(detection)) {
            RCLCPP_INFO(get_logger(), "Mine detection reported: ID=%d", msg.mine_id);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to send mine detection: %s",
                         reporter_.getLastError().c_str());
        }
    }
    
    void onClearanceReport(const perimeter_msgs::msg::ClearanceReport& msg) {
        ClearanceReport report;
        report.mine_id = msg.mine_id;
        report.x = msg.x;
        report.y = msg.y;
        report.method = msg.method;
        report.success = msg.success;
        report.details = msg.details;
        report.timestamp = std::to_string(msg.timestamp.sec) + "." + std::to_string(msg.timestamp.nanosec);
        
        if (reporter_.sendClearanceReport(report)) {
            RCLCPP_INFO(get_logger(), "Clearance report sent: mine_id=%d success=%d",
                        msg.mine_id, msg.success);
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to send clearance report: %s",
                         reporter_.getLastError().c_str());
        }
    }
    
    void onMissionSummary(const perimeter_msgs::msg::MissionSummary& msg) {
        MissionSummary summary;
        summary.scenario_name = msg.scenario_name;
        summary.result = msg.result;
        summary.reason = msg.reason;
        summary.total_waypoints = msg.total_waypoints;
        summary.waypoints_completed = msg.waypoints_completed;
        summary.mines_detected = msg.mines_detected;
        summary.mines_cleared = msg.mines_cleared;
        summary.mission_duration = msg.mission_duration;
        summary.coverage_percent = msg.coverage_percent;
        summary.start_time = std::to_string(msg.start_time.sec);
        summary.end_time = std::to_string(msg.end_time.sec);
        
        if (reporter_.sendMissionSummary(summary)) {
            RCLCPP_INFO(get_logger(), "Mission summary sent: result=%s", msg.result.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to send mission summary: %s",
                         reporter_.getLastError().c_str());
        }
    }
    
    std::string api_endpoint_;
    HttpReporter reporter_;
    
    rclcpp::Subscription<perimeter_msgs::msg::PerimeterStatus>::SharedPtr movement_sub_;
    rclcpp::Subscription<perimeter_msgs::msg::MineDetection>::SharedPtr mine_sub_;
    rclcpp::Subscription<perimeter_msgs::msg::ClearanceReport>::SharedPtr clearance_sub_;
    rclcpp::Subscription<perimeter_msgs::msg::MissionSummary>::SharedPtr summary_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_status_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReporterNode>());
    rclcpp::shutdown();
    return 0;
}
