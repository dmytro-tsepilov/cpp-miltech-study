#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include "antidrone_turret/msg/servo_command.hpp"

class YawServoDriverNode : public rclcpp::Node {
public:
  YawServoDriverNode()
    : Node("yaw_servo_driver_node")
  {
    sub_ = create_subscription<antidrone_turret::msg::ServoCommand>(
      "/servo/cmd", 10,
      [this](const antidrone_turret::msg::ServoCommand::SharedPtr msg) {
        on_command(msg);
      });

    RCLCPP_INFO(get_logger(), "yaw_servo_driver_node started, subscribing to /servo/cmd");
  }

private:
  void on_command(const antidrone_turret::msg::ServoCommand::SharedPtr msg)
  {
    const char* dir_str = "UNKNOWN";
    switch (msg->direction) {
      case antidrone_turret::msg::ServoCommand::LEFT:
        dir_str = "LEFT";
        break;
      case antidrone_turret::msg::ServoCommand::CENTER:
        dir_str = "CENTER";
        break;
      case antidrone_turret::msg::ServoCommand::RIGHT:
        dir_str = "RIGHT";
        break;
    }

    std::ostringstream oss;
    oss << "yaw_servo_driver_node received: direction=" << dir_str
        << " target_x=" << msg->target_x
        << " error_x=" << msg->error_x;
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  rclcpp::Subscription<antidrone_turret::msg::ServoCommand>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
