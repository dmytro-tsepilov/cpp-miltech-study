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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "perimeter_miner/mode_switch.hpp"
#include "perimeter_msgs/msg/perimeter_status.hpp"
#include "perimeter_msgs/srv/switch_mode.hpp"

using namespace std::chrono_literals;
using perimeter_miner::ControlMode;
using perimeter_miner::ModeSwitch;
using perimeter_miner::controlModeFromUint8;
using perimeter_miner::controlModeToString;

/// Mode switch node - handles mode transitions and operator override
class ModeSwitchNode : public rclcpp::Node
{
public:
  ModeSwitchNode()
  : Node("mode_switch_node")
  , mode_switch_()
  {
    RCLCPP_INFO(get_logger(), "ModeSwitchNode starting...");

    // Publishers
    status_pub_ = create_publisher<perimeter_msgs::msg::PerimeterStatus>(
      "/control/status", 10);

    // Service for mode switching
    switch_srv_ = create_service<perimeter_msgs::srv::SwitchMode>(
      "/control/switch_mode",
      [this](
        const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Request> req,
        const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Response> res) {
        onSwitchRequest(req, res);
      });

    // Timer for status publishing
    status_timer_ = create_wall_timer(500ms, [this]() { publishStatus(); });

    RCLCPP_INFO(get_logger(), "Current mode: %s",
                controlModeToString(mode_switch_.getCurrentMode()));
  }

private:
  void onSwitchRequest(
    const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Request> req,
    const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Response> res)
  {
    ControlMode requested = controlModeFromUint8(req->mode);
    RCLCPP_INFO(get_logger(), "Mode switch request: %s -> %s",
                controlModeToString(mode_switch_.getCurrentMode()),
                controlModeToString(requested));

    // Operator override for TELEOP mode (priority)
    if (requested == ControlMode::TELEOP) {
      res->success = mode_switch_.operatorOverride();
    } else {
      res->success = mode_switch_.requestMode(requested);
      if (res->success) {
        mode_switch_.applyRequest();
      }
    }

    res->message = mode_switch_.getLastMessage();

    RCLCPP_INFO(get_logger(), "Result: %s (%s)",
                res->success ? "SUCCESS" : "FAILED",
                res->message.c_str());
  }

  void publishStatus()
  {
    auto msg = perimeter_msgs::msg::PerimeterStatus();
    msg.mode = static_cast<uint8_t>(mode_switch_.getCurrentMode());
    msg.timestamp = now();
    status_pub_->publish(msg);
  }

  ModeSwitch mode_switch_;

  rclcpp::Publisher<perimeter_msgs::msg::PerimeterStatus>::SharedPtr status_pub_;
  rclcpp::Service<perimeter_msgs::srv::SwitchMode>::SharedPtr switch_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeSwitchNode>());
  rclcpp::shutdown();
  return 0;
}
