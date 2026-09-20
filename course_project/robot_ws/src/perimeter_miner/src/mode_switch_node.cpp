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

/// Mode switch node - syncs mode state with miner_node via /control/status topic
class ModeSwitchNode : public rclcpp::Node
{
public:
ModeSwitchNode()
  : Node("mode_switch_node")
  , mode_switch_()
{
  RCLCPP_INFO(get_logger(), "ModeSwitchNode starting...");

  // Publishers - publish current mode state to /control/status
  status_pub_ = create_publisher<perimeter_msgs::msg::PerimeterStatus>(
    "/control/status", 10);

  // Service for mode switching - forwards to miner_node via topic
  switch_srv_ = create_service<perimeter_msgs::srv::SwitchMode>(
    "/control/switch_mode",
    [this](
      const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Request> req,
      const std::shared_ptr<perimeter_msgs::srv::SwitchMode::Response> res) {
      onSwitchRequest(req, res);
    });

  // Subscribe to miner_node status topic to sync mode state
  // (miner_node publishes current mode on /control/status)
  mode_sub_ = create_subscription<perimeter_msgs::msg::PerimeterStatus>(
    "/perimeter/status", 10,
    [this](const perimeter_msgs::msg::PerimeterStatus::SharedPtr msg) {
      ControlMode new_mode = controlModeFromUint8(msg->mode);
      if (new_mode != mode_switch_.getCurrentMode()) {
        mode_switch_.setMode(new_mode);
        RCLCPP_INFO(get_logger(), "Synced mode from miner_node: %s",
                    controlModeToString(new_mode));
      }
    });

  // Timer for status publishing
  status_timer_ = create_wall_timer(500ms, [this]() {
      publishStatus();
    });

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

  // Update local state and publish to topic for miner_node to receive
  mode_switch_.setMode(requested);
  res->success = true;
  res->message = "Mode set to " + std::string(controlModeToString(requested)) +
                 " - waiting for miner_node confirmation";

  // Publish immediately so miner_node receives it
  auto msg = perimeter_msgs::msg::PerimeterStatus();
  msg.mode = static_cast<uint8_t>(requested);
  msg.timestamp = now();
  status_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), "Published mode change to /control/status");
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
rclcpp::Subscription<perimeter_msgs::msg::PerimeterStatus>::SharedPtr mode_sub_;
rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeSwitchNode>());
  rclcpp::shutdown();
  return 0;
}
