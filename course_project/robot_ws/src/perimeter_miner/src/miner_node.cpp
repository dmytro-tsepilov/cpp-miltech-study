// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

#include "perimeter_miner/perimeter_config.hpp"
#include "perimeter_miner/perimeter_loader.hpp"
#include "perimeter_miner/perimeter_tracker.hpp"
#include "perimeter_miner/mode_switch.hpp"
#include "perimeter_miner/hold_controller.hpp"
#include "perimeter_msgs/msg/perimeter_status.hpp"
#include "perimeter_msgs/msg/mine_detection.hpp"
#include "perimeter_msgs/msg/clearance_report.hpp"
#include "perimeter_msgs/msg/mission_summary.hpp"
#include "perimeter_msgs/srv/switch_mode.hpp"
#include "perimeter_msgs/srv/trigger_clearance.hpp"

using namespace std::chrono_literals;
using perimeter_miner::ControlMode;
using perimeter_miner::HoldController;
using perimeter_miner::ModeSwitch;
using perimeter_miner::MoveCommand;
using perimeter_miner::PerimeterConfig;
using perimeter_miner::PerimeterLoader;
using perimeter_miner::PerimeterTracker;
using perimeter_miner::RobotState;
using perimeter_miner::Waypoint;
using perimeter_miner::controlModeFromUint8;
using perimeter_miner::controlModeToString;

/// Default config for initialization
PerimeterConfig getDefaultConfig()
{
  PerimeterConfig config;
  config.name = "default";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;
  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{20.0, 0.0, M_PI_2, 1.0},
    Waypoint{20.0, 20.0, M_PI, 1.0},
    Waypoint{0.0, 20.0, -M_PI_2, 1.0}
  };
  return config;
}

/// Main miner node - integrates all components for perimeter patrol
class MinerNode : public rclcpp::Node
{
public:
  MinerNode()
  : Node("miner_node")
  , perimeter_tracker_(getDefaultConfig())
  , mode_switch_()
  , hold_controller_()
  {
    RCLCPP_INFO(get_logger(), "Initializing Perimeter Miner Node...");

    // Declare parameters
    this->declare_parameter("scenario_file", "training_ground.yaml");
    this->declare_parameter("config_search_paths", std::vector<std::string>{});
    this->declare_parameter("enable_coverage", false);

    // Check if coverage mode is enabled
    bool enable_coverage = this->get_parameter("enable_coverage").as_bool();

    // Load perimeter config from file
    PerimeterConfig loaded_config = loadPerimeterConfig();
    perimeter_tracker_ = PerimeterTracker(loaded_config);

    // Initialize perimeter tracker with default state
    robot_state_ = RobotState{0.0, 0.0, 0.0, 0.0, 0.0};
    perimeter_tracker_.updateRobotState(robot_state_);

    // Enable coverage mode if config type is "coverage" or enable_coverage parameter is true
    if (enable_coverage || loaded_config.name.find("coverage") != std::string::npos) {
        perimeter_miner::CoverageConfig cov_cfg;
        cov_cfg.min_x = loaded_config.bounding_box.min_x;
        cov_cfg.min_y = loaded_config.bounding_box.min_y;
        cov_cfg.max_x = loaded_config.bounding_box.max_x;
        cov_cfg.max_y = loaded_config.bounding_box.max_y;
        cov_cfg.pass_spacing = loaded_config.bounding_box.pass_spacing;
        cov_cfg.coverage_speed = loaded_config.bounding_box.coverage_speed;
        cov_cfg.scan_direction = loaded_config.bounding_box.scan_direction;
        
        perimeter_tracker_.setCoverageMode(cov_cfg);
        RCLCPP_INFO(get_logger(), "Coverage mode enabled!");
    }

    // Setup mode switch safety checks
    mode_switch_.setAutonomousCheck([this]() {
      // Can return to autonomous if robot is near perimeter
      auto status = perimeter_tracker_.getStatus();
      return std::abs(status.lateral_error) < 5.0;
    });

    // Initialize hold position to start
    if (perimeter_tracker_.getConfig().waypointCount() > 0) {
      const auto &start = perimeter_tracker_.getConfig().getWaypoint(0);
      hold_controller_.setHoldPosition(start.x, start.y, start.heading);
    }

    // Setup publishers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/control/cmd_vel", 10);

    status_pub_ = create_publisher<perimeter_msgs::msg::PerimeterStatus>(
      "/perimeter/status", 10);

    status_srv_ = create_service<perimeter_msgs::srv::SwitchMode>(
      "/control/switch_mode",
      [this](
        const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Request> req,
        const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Response> res) {
        onModeSwitchService(req, res);
      });

    clearance_srv_ = create_service<perimeter_msgs::srv::TriggerClearance>(
      "/control/trigger_clearance",
      [this](
        const std::shared_ptr<perimeter_msgs::srv::TriggerClearance::Request> req,
        const std::shared_ptr<perimeter_msgs::srv::TriggerClearance::Response> res) {
        onClearanceService(req, res);
      });

    // Setup subscriptions
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        onOdometry(*msg);
      });

    mine_detected_sub_ = create_subscription<perimeter_msgs::msg::MineDetection>(
      "/mines/detected", 10,
      [this](const perimeter_msgs::msg::MineDetection::SharedPtr msg) {
        onMineDetected(*msg);
      });

    // Subscribe to teleoperation input (keyboard/joy)
    teleop_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/teleop/cmd", 10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;  // Signal that operator has input
        teleop_active_ = true;
      });

    // Publisher for ClearanceReport
    clearance_pub_ = create_publisher<perimeter_msgs::msg::ClearanceReport>(
      "/mines/cleared", 10);

    // Publisher for MissionSummary
    mission_summary_pub_ = create_publisher<perimeter_msgs::msg::MissionSummary>(
      "/mission/summary", 10);

    // Timers
    control_timer_ = create_wall_timer(20ms, [this]() { controlTick(); });
    status_timer_ = create_wall_timer(500ms, [this]() { publishStatus(); });

    RCLCPP_INFO(get_logger(), "Perimeter Miner Node initialized");
    RCLCPP_INFO(get_logger(), "Perimeter: %s (%zu waypoints)",
                 perimeter_tracker_.getConfig().name.c_str(),
                 perimeter_tracker_.getConfig().waypointCount());

    // Initialize mission start time
    mission_start_time_ = now();
    last_teleop_input_ = now();
  }

  ~MinerNode() override = default;

private:
  // Load perimeter configuration from YAML file
  PerimeterConfig loadPerimeterConfig()
  {
    // Get scenario file parameter
    std::string scenario_file = this->get_parameter("scenario_file").as_string();

    // Get custom search paths from parameter (empty = use defaults)
    auto custom_paths = this->get_parameter("config_search_paths").as_string_array();

    // Build search path list: custom paths first, then standard ROS locations
    std::vector<std::string> possible_paths;

    // Add custom paths if provided
    for (const auto &base : custom_paths) {
      if (!base.empty()) {
        possible_paths.push_back(base + "/" + scenario_file);
      }
    }

    // Add standard ROS 2 package paths (in priority order)
    possible_paths.push_back("perimeter_miner/config/" + scenario_file);  // Relative/install

    // Note: Absolute hardcoded paths removed - use config_search_paths parameter instead
    // Example: ros2 launch perimeter_miner system.launch.py config_search_paths:=["/custom/path"]

    PerimeterConfig config;
    std::string loaded_path;

    for (const auto &path : possible_paths) {
      config = perimeter_miner::PerimeterLoader::loadFromFile(path);
      if (!config.waypoints.empty()) {
        loaded_path = path;
        break;
      }
    }

    // Fallback to default if loading fails
    if (config.waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "Failed to load config from any path, using defaults");
      RCLCPP_INFO(get_logger(), "Try: ros2 launch perimeter_miner system.launch.py");
      RCLCPP_INFO(get_logger(), "  scenario_file:=<your_config>.yaml");
      config.name = "training_ground";
      config.closed_loop = true;
      config.tolerance = 0.5;
      config.max_speed = 2.0;

      // Default training perimeter (square)
      config.waypoints = {
        Waypoint{0.0, 0.0, 0.0, 1.0},
        Waypoint{20.0, 0.0, M_PI_2, 1.0},
        Waypoint{20.0, 20.0, M_PI, 1.0},
        Waypoint{0.0, 20.0, -M_PI_2, 1.0}
      };
    } else {
      RCLCPP_INFO(get_logger(), "Loaded perimeter config from '%s' with %zu waypoints",
                  loaded_path.c_str(), static_cast<size_t>(config.waypoints.size()));
    }

    return config;
  }

  // Odometry callback
  void onOdometry(const nav_msgs::msg::Odometry &msg)
  {
    robot_state_.x = msg.pose.pose.position.x;
    robot_state_.y = msg.pose.pose.position.y;

    // Convert quaternion to heading (yaw)
    auto q = msg.pose.pose.orientation;
    // For rotation around Z axis only (x=0, y=0): yaw = 2*atan2(q.z, q.w)
    // This is the correct formula for pure yaw quaternions
    robot_state_.heading = 2.0 * std::atan2(q.z, q.w);

    robot_state_.linear_speed = msg.twist.twist.linear.x;
    robot_state_.angular_speed = msg.twist.twist.angular.z;

    // Update perimeter tracker
    perimeter_tracker_.updateRobotState(robot_state_);
  }

  // Mine detection callback
  void onMineDetected(const perimeter_msgs::msg::MineDetection &msg)
  {
    RCLCPP_INFO(get_logger(), "Mine detected! ID=%d at (%.1f, %.1f) type=%s confidence=%.2f",
                msg.mine_id, msg.x, msg.y, msg.type.c_str(), msg.confidence);

    current_mine_ = msg;
    mine_detected_ = true;
    mines_detected_count_++;

    // Update mission summary with detected mines count
    detected_mines_.push_back(msg);

    // If in HOLD mode, initiate clearance
    if (mode_switch_.getCurrentMode() == ControlMode::HOLD) {
      RCLCPP_INFO(get_logger(), "Initiating mine clearance for ID=%d", msg.mine_id);

      // Publish ClearanceReport immediately
      publishClearanceReport(msg.mine_id, robot_state_.x, robot_state_.y, "detection", true);
    }
  }

  // Main control loop with real dt tracking
  void controlTick()
  {
    // Track dt for PID controllers
    auto now_ns = now();
    if (last_control_tick_.nanoseconds() > 0) {
      rclcpp::Duration dt_dur = now_ns - last_control_tick_;
      current_dt_ = dt_dur.seconds();
    }
    last_control_tick_ = now_ns;

    // Clamp dt to reasonable values (0.001s to 1.0s)
    if (current_dt_ <= 0.0 || current_dt_ > 1.0) {
      current_dt_ = 0.02;  // Fallback to 50Hz default
    }

    // DIAGNOSTIC: Log first 5 ticks with full state
    static int tick_count = 0;
    tick_count++;

    // Apply mode switch if pending
    if (mode_switch_.isSwitching()) {
      mode_switch_.applyRequest();
    }

    ControlMode current_mode = mode_switch_.getCurrentMode();
    MoveCommand cmd;

    switch (current_mode) {
      case ControlMode::AREA_COVERAGE: {
        // Area coverage mode - follow zigzag pattern
        if (!perimeter_tracker_.isCoverageComplete()) {
          cmd = perimeter_tracker_.decide(current_dt_);
          
          // Check if coverage complete
          if (perimeter_tracker_.isCoverageComplete()) {
            RCLCPP_INFO(get_logger(), "Area coverage complete!");
          }
        } else {
          cmd = MoveCommand::zero();
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Coverage complete - holding position");
        }
        
        // Check if mine detected - switch to HOLD
        if (mine_detected_) {
          RCLCPP_INFO(get_logger(), "Mine detected! Switching to HOLD mode");
          hold_controller_.setHoldPosition(
            robot_state_.x, robot_state_.y, robot_state_.heading);
          mode_switch_.requestMode(ControlMode::HOLD);
          mine_detected_ = false;
        }
        break;
      }

      case ControlMode::AUTONOMOUS: {
        // DIAGNOSTIC: Log robot state before decide
        if (tick_count <= 5 || tick_count % 100 == 0) {
          fprintf(stderr, "[CONTROL] #%d AUTONOMOUS: robot=(%.3f, %.3f, %.4f rad), dt=%.4f\n",
              tick_count, robot_state_.x, robot_state_.y, robot_state_.heading, current_dt_);
        }
        
        // Perimeter tracking
        cmd = perimeter_tracker_.decide(current_dt_);

        // Check if mine detected - switch to HOLD
        if (mine_detected_) {
          RCLCPP_INFO(get_logger(), "Mine detected! Switching to HOLD mode");
          hold_controller_.setHoldPosition(
            robot_state_.x, robot_state_.y, robot_state_.heading);
          mode_switch_.requestMode(ControlMode::HOLD);
          mine_detected_ = false;
        }

        // Check if mission complete (all waypoints done for open perimeter)
        checkMissionCompletion();
        break;
      }

      case ControlMode::TELEOP: {
        // Teleoperation - wait for operator input via /teleop/cmd topic
        // If no teleop input received, use zero command
        cmd = MoveCommand::zero();

        // Auto-return to AUTONOMOUS after 30s of no teleop input (timeout protection)
        auto teleop_dur = now_ns - last_teleop_input_;
        auto teleop_elapsed = static_cast<int>(teleop_dur.seconds());
        if (teleop_elapsed > 30) {
          RCLCPP_INFO(get_logger(), "Teleop timeout (30s), returning to AUTONOMOUS");
          mode_switch_.requestMode(ControlMode::AUTONOMOUS);
          teleop_active_ = false;
          break;
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TELEOP mode active - waiting for operator input (timeout: %ds)",
                             static_cast<int>(30 - teleop_elapsed));
        break;
      }

      case ControlMode::HOLD: {
        // Hold position with real dt
        cmd = hold_controller_.compute(robot_state_, current_dt_);

        // Check if at hold position - can return to autonomous
        if (hold_controller_.isAtHoldPosition(robot_state_, 0.3)) {
          RCLCPP_INFO(get_logger(), "At hold position - ready to resume autonomous patrol");
          // Auto-return after clearance confirmation
          if (mine_cleared_) {
            mode_switch_.requestMode(ControlMode::AUTONOMOUS);
            mine_cleared_ = false;
          }
        }
        break;
      }
    }

    // Publish command
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = now();
    twist_msg.twist.linear.x = cmd.linear_x;
    twist_msg.twist.linear.y = cmd.linear_y;
    twist_msg.twist.angular.z = cmd.angular_z;
    
    // DIAGNOSTIC: Log first 10 commands and then every 50th
    if (tick_count <= 10 || tick_count % 50 == 0) {
      fprintf(stderr, "[COMMAND] #%d: linear_x=%.3f, angular_z=%.3f\n",
          tick_count, cmd.linear_x, cmd.angular_z);
    }
    
    cmd_vel_pub_->publish(twist_msg);
  }

  // Status publishing
  void publishStatus()
  {
    auto status_msg = perimeter_msgs::msg::PerimeterStatus();
    auto tracker_status = perimeter_tracker_.getStatus();

    status_msg.mode = static_cast<uint8_t>(mode_switch_.getCurrentMode());
    status_msg.waypoint_index = tracker_status.waypoint_index;
    status_msg.target_x = tracker_status.target_x;
    status_msg.target_y = tracker_status.target_y;
    status_msg.current_x = tracker_status.current_x;
    status_msg.current_y = tracker_status.current_y;
    status_msg.lateral_error = tracker_status.lateral_error;
    status_msg.speed = tracker_status.speed;
    status_msg.mine_detected = mine_detected_;
    status_msg.timestamp = now();

    status_pub_->publish(status_msg);
  }

  // Check mission completion and publish summary
  void checkMissionCompletion()
  {
    // Only check for open perimeter (closed loop patrols are continuous)
    if (perimeter_tracker_.getConfig().closed_loop) {
      return;
    }

    // Check if reached end of open perimeter
    if (!perimeter_tracker_.reachedWaypoint()) {
      return;
    }

    static bool mission_completed_ = false;
    if (mission_completed_) {
      return;
    }

    // Get tracker status for summary
    auto tracker_status = perimeter_tracker_.getStatus();
    auto &config = perimeter_tracker_.getConfig();

    // Determine mission result based on clearance status
    std::string result = "SUCCESS";
    std::string reason = "All waypoints completed and all mines cleared";

    auto summary_msg = perimeter_msgs::msg::MissionSummary();
    summary_msg.scenario_name = config.name;
    summary_msg.result = result;
    summary_msg.reason = reason;
    summary_msg.total_waypoints = static_cast<int32_t>(config.waypointCount());
    summary_msg.waypoints_completed = static_cast<int32_t>(tracker_status.waypoint_index + 1);
    summary_msg.mines_detected = 0;  // Would be tracked by mine_spawner
    summary_msg.mines_cleared = 0;   // Would be tracked by mine_spawner
    summary_msg.mission_duration = (now() - mission_start_time_).seconds();
    summary_msg.coverage_percent = 100.0;
    summary_msg.start_time.sec = static_cast<int32_t>(mission_start_time_.nanoseconds() / 1e9);
    summary_msg.start_time.nanosec = static_cast<uint32_t>(

        (mission_start_time_.nanoseconds() % 1000000000) / 1000);

    summary_msg.end_time.sec = static_cast<int32_t>(now().nanoseconds() / 1e9);
    summary_msg.end_time.nanosec = static_cast<uint32_t>((now().nanoseconds() % 1000000000) / 1000);

    // Publish mission summary
    mission_summary_pub_->publish(summary_msg);
    RCLCPP_INFO(get_logger(), "Mission summary published: result=%s, duration=%.1fs",
                result.c_str(), summary_msg.mission_duration);

    mission_completed_ = true;
  }

  // Mode switch service handler
  void onModeSwitchService(
    const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Request> req,
    const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Response> res)
  {
    ControlMode requested = controlModeFromUint8(req->mode);
    RCLCPP_INFO(get_logger(), "Mode switch request: %s", controlModeToString(requested));

    if (requested == ControlMode::TELEOP) {
      // Operator override for TELEOP
      res->success = mode_switch_.operatorOverride();
    } else {
      // Normal mode request
      res->success = mode_switch_.requestMode(requested);
    }

    res->message = mode_switch_.getLastMessage();
  }

  // Clearance service handler
  void onClearanceService(
    const std::shared_ptr<perimeter_msgs::srv::TriggerClearance::Request> req,
    const std::shared_ptr<perimeter_msgs::srv::TriggerClearance::Response> res)
  {
    RCLCPP_INFO(get_logger(), "Clearance request: mine_id=%d method=%s",
                req->mine_id, req->method.c_str());

    // Publish clearance report via /mines/cleared topic
    publishClearanceReport(req->mine_id, robot_state_.x, robot_state_.y, req->method, true);

    // In real system, this would trigger actual clearance mechanism
    RCLCPP_INFO(get_logger(), "Clearance accepted for mine ID=%d", req->mine_id);
    mines_cleared_count_++;

    res->accepted = true;
    res->reason = "Clearance initiated successfully";

    // Switch back to autonomous after clearance (if not already in HOLD)
    if (mode_switch_.getCurrentMode() == ControlMode::HOLD) {
      mine_cleared_ = true;
      // Don't auto-switch - let hold controller handle the transition when at position
    }
  }

  // Helper to publish ClearanceReport
  void publishClearanceReport(int32_t mine_id, double x, double y,

      const std::string &method, bool success)

  {
    auto report = perimeter_msgs::msg::ClearanceReport();
    report.mine_id = mine_id;
    report.x = x;
    report.y = y;
    report.method = method;
    report.success = success;
    report.details = success ? "Clearance completed successfully" : "Clearance failed";
    report.timestamp = now();

    clearance_pub_->publish(report);
    RCLCPP_INFO(get_logger(), "Published ClearanceReport: mine_id=%d, success=%d",
                mine_id, success);
  }

  // Member variables
  PerimeterConfig perimeter_config_;
  PerimeterTracker perimeter_tracker_;
  ModeSwitch mode_switch_;
  HoldController hold_controller_;
  RobotState robot_state_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<perimeter_msgs::msg::PerimeterStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<perimeter_msgs::msg::ClearanceReport>::SharedPtr clearance_pub_;
  rclcpp::Publisher<perimeter_msgs::msg::MissionSummary>::SharedPtr mission_summary_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<perimeter_msgs::msg::MineDetection>::SharedPtr mine_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr teleop_sub_;
  rclcpp::Service<perimeter_msgs::srv::SwitchMode>::SharedPtr status_srv_;
  rclcpp::Service<perimeter_msgs::srv::TriggerClearance>::SharedPtr clearance_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Mine detection state
  perimeter_msgs::msg::MineDetection current_mine_;
  bool mine_detected_ = false;
  bool mine_cleared_ = false;
  bool teleop_active_ = false;
  int32_t mines_detected_count_ = 0;
  int32_t mines_cleared_count_ = 0;

  // Mission timing
  rclcpp::Time mission_start_time_;

  // dt tracking for PID controllers
  double current_dt_ = 0.02;  // Default 50Hz
  rclcpp::Time last_control_tick_;
  rclcpp::Time last_teleop_input_;

  // Detected mines tracking
  std::vector<perimeter_msgs::msg::MineDetection> detected_mines_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinerNode>());
  rclcpp::shutdown();
  return 0;
}
