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

    // Declare scenario_file parameter
    this->declare_parameter("scenario_file", "training_ground.yaml");

    // Load perimeter config from file
    PerimeterConfig loaded_config = loadPerimeterConfig();
    perimeter_tracker_ = PerimeterTracker(loaded_config);

    // Initialize perimeter tracker with default state
    robot_state_ = RobotState{0.0, 0.0, 0.0, 0.0, 0.0};
    perimeter_tracker_.updateRobotState(robot_state_);

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

    // Timers
    control_timer_ = create_wall_timer(20ms, [this]() { controlTick(); });
    status_timer_ = create_wall_timer(500ms, [this]() { publishStatus(); });

    RCLCPP_INFO(get_logger(), "Perimeter Miner Node initialized");
    RCLCPP_INFO(get_logger(), "Perimeter: %s (%zu waypoints)",
                 perimeter_tracker_.getConfig().name.c_str(),
                 perimeter_tracker_.getConfig().waypointCount());
  }

  ~MinerNode() override = default;

private:
  // Load perimeter configuration from YAML file
  PerimeterConfig loadPerimeterConfig()
  {
    // Get scenario file parameter
    std::string scenario_file = this->get_parameter("scenario_file").as_string();

    // Try to load from multiple possible locations
    std::vector<std::string> possible_paths = {
      "perimeter_miner/config/" + scenario_file,  // Relative path
      "/home/dimon/cpp-miltech-study/homework_14/course_project/robot_ws/src/"
        "perimeter_miner/config/" + scenario_file,  // Absolute source path
      "/home/dimon/cpp-miltech-study/homework_14/course_project/robot_ws/install/"
        "perimeter_miner/share/perimeter_miner/config/" + scenario_file  // Install prefix
    };

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
    double siny_uncoupled = -2.0 * (q.y * q.w - q.z * q.x);
    double cosy_uncoupled = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    robot_state_.heading = std::atan2(siny_uncoupled, cosy_uncoupled);

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

    // If in HOLD mode, initiate clearance
    if (mode_switch_.getCurrentMode() == ControlMode::HOLD) {
      RCLCPP_INFO(get_logger(), "Initiating mine clearance for ID=%d", msg.mine_id);
    }
  }

  // Main control loop
  void controlTick()
  {
    // Apply mode switch if pending
    if (mode_switch_.isSwitching()) {
      mode_switch_.applyRequest();
    }

    ControlMode current_mode = mode_switch_.getCurrentMode();
    MoveCommand cmd;

    switch (current_mode) {
      case ControlMode::AUTONOMOUS: {
        // Perimeter tracking
        cmd = perimeter_tracker_.decide();

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

      case ControlMode::TELEOP: {
        // Teleoperation - zero command (wait for operator input)
        cmd = MoveCommand::zero();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TELEOP mode active - waiting for operator input");
        break;
      }

      case ControlMode::HOLD: {
        // Hold position
        cmd = hold_controller_.compute(robot_state_);

        // Check if at hold position - can return to autonomous
        if (hold_controller_.isAtHoldPosition(robot_state_, 0.3)) {
          RCLCPP_INFO(get_logger(), "At hold position - ready to resume autonomous patrol");
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

    // Publish clearance report
    auto report = perimeter_msgs::msg::ClearanceReport();
    report.mine_id = req->mine_id;
    report.x = robot_state_.x;
    report.y = robot_state_.y;
    report.method = req->method;
    report.success = true;
    report.details = "Clearance initiated";
    report.timestamp = now();

    // In real system, this would trigger actual clearance mechanism
    RCLCPP_INFO(get_logger(), "Clearance accepted for mine ID=%d", req->mine_id);

    res->accepted = true;
    res->reason = "Clearance initiated successfully";

    // Switch back to autonomous after clearance
    mode_switch_.requestMode(ControlMode::AUTONOMOUS);
  }

  // Member variables
  PerimeterConfig perimeter_config_;
  PerimeterTracker perimeter_tracker_;
  ModeSwitch mode_switch_;
  HoldController hold_controller_;
  RobotState robot_state_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<perimeter_msgs::msg::PerimeterStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<perimeter_msgs::msg::MineDetection>::SharedPtr mine_detected_sub_;
  rclcpp::Service<perimeter_msgs::srv::SwitchMode>::SharedPtr status_srv_;
  rclcpp::Service<perimeter_msgs::srv::TriggerClearance>::SharedPtr clearance_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Mine detection state
  perimeter_msgs::msg::MineDetection current_mine_;
  bool mine_detected_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinerNode>());
  rclcpp::shutdown();
  return 0;
}
