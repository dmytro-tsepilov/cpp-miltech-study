#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include "antidrone_turret/msg/gimbal_command.hpp"

class GimbalDriverNode : public rclcpp::Node {
public:
  GimbalDriverNode()
    : Node("gimbal_driver_node")
  {
    sub_ = create_subscription<antidrone_turret::msg::GimbalCommand>(
      "/gimbal/cmd", 10,
      [this](const antidrone_turret::msg::GimbalCommand::SharedPtr msg) {
        on_command(msg);
      });

    RCLCPP_INFO(get_logger(), "gimbal_driver_node started, subscribing to /gimbal/cmd");
  }

private:
  void on_command(const antidrone_turret::msg::GimbalCommand::SharedPtr msg)
  {
    const char* dir_str = "UNKNOWN";
    switch (msg->direction) {
      case antidrone_turret::msg::GimbalCommand::DOWN:
        dir_str = "DOWN";
        break;
      case antidrone_turret::msg::GimbalCommand::CENTER:
        dir_str = "CENTER";
        break;
      case antidrone_turret::msg::GimbalCommand::UP:
        dir_str = "UP";
        break;
    }

    std::ostringstream oss;
    oss << "gimbal_driver_node received: direction=" << dir_str
        << " target_y=" << msg->target_y
        << " error_y=" << msg->error_y;
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  rclcpp::Subscription<antidrone_turret::msg::GimbalCommand>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}
