#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"
#include "antidrone_turret/turret_controller.hpp"

class TurretControllerNode : public rclcpp::Node {
public:
  TurretControllerNode()
    : Node("turret_controller_node")
  {
    // Declare parameters
    confidence_threshold_ = declare_parameter<double>(
      "confidence_threshold", 0.80);
    max_distance_m_ = declare_parameter<double>(
      "max_distance_m", 30.0);

    config_ = antidrone_turret::ControllerConfig{
      static_cast<float>(confidence_threshold_),
      static_cast<float>(max_distance_m_)
    };

    // Publishers
    gimbal_pub_ = create_publisher<antidrone_turret::msg::GimbalCommand>(
      "/gimbal/cmd", 10);
    servo_pub_ = create_publisher<antidrone_turret::msg::ServoCommand>(
      "/servo/cmd", 10);
    status_pub_ = create_publisher<antidrone_turret::msg::TurretStatus>(
      "/turret/status", 10);

    // Subscriber for target perception
    target_sub_ = create_subscription<antidrone_turret::msg::Target>(
      "/perception/target", 10,
      [this](const antidrone_turret::msg::Target::SharedPtr msg) {
        on_target(msg);
      });

    // Subscriber for actuator status
    actuator_sub_ = create_subscription<antidrone_turret::msg::ActuatorStatus>(
      "/actuator/status", 10,
      [this](const antidrone_turret::msg::ActuatorStatus::SharedPtr msg) {
        on_actuator_status(msg);
      });

    // Client for trigger service
    trigger_client_ = create_client<antidrone_turret::srv::TriggerActuator>(
      "/actuator/trigger");

    RCLCPP_INFO(get_logger(), "turret_controller_node started");
    RCLCPP_INFO(get_logger(), "  confidence_threshold=%.2f, max_distance_m=%.1f",
                confidence_threshold_, max_distance_m_);
  }

private:
  void on_target(const antidrone_turret::msg::Target::SharedPtr msg)
  {
    // Build target input
    antidrone_turret::TargetInput target_input{};
    target_input.visible = msg->visible;
    target_input.x = msg->x;
    target_input.y = msg->y;
    target_input.distance_m = msg->distance_m;
    target_input.confidence = msg->confidence;

    // Build actuator input
    antidrone_turret::ActuatorInput actuator_input{};
    actuator_input.ready = last_actuator_ready_;
    actuator_input.trigger_count = last_trigger_count_;

    // Process controller logic
    const auto output = antidrone_turret::process_controller(
      target_input, actuator_input, config_);

    // Publish TurretStatus always
    {
      auto status_msg = antidrone_turret::msg::TurretStatus{};
      status_msg.target_state = output.status.target_state;
      status_msg.action = output.status.action;
      status_msg.trigger_state = output.status.trigger_state;
      status_msg.confidence = output.status.confidence;
      status_msg.distance_m = output.status.distance_m;
      status_pub_->publish(status_msg);
    }

    // Publish gimbal/servo commands only when tracking
    if (output.status.action == static_cast<std::uint8_t>(
          antidrone_turret::ActionState::kTrack)) {
      auto gimbal_msg = antidrone_turret::msg::GimbalCommand{};
      gimbal_msg.direction = output.gimbal_cmd.direction;
      gimbal_msg.target_y = output.gimbal_cmd.target_y;
      gimbal_msg.error_y = output.gimbal_cmd.error_y;
      gimbal_pub_->publish(gimbal_msg);

      auto servo_msg = antidrone_turret::msg::ServoCommand{};
      servo_msg.direction = output.servo_cmd.direction;
      servo_msg.target_x = output.servo_cmd.target_x;
      servo_msg.error_x = output.servo_cmd.error_x;
      servo_pub_->publish(servo_msg);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "TRACK: x=%.1f y=%.1f dist=%.1f conf=%.2f gimbal_dir=%d servo_dir=%d",
        target_input.x, target_input.y, target_input.distance_m,
        target_input.confidence, output.gimbal_cmd.direction,
        output.servo_cmd.direction);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "IDLE: visible=%s conf=%.2f dist=%.1f",
        target_input.visible ? "true" : "false",
        target_input.confidence, target_input.distance_m);
    }

    // Call trigger service if needed
    if (output.should_trigger) {
      call_trigger_service(output.trigger_confidence, output.trigger_distance);
    }
  }

  void on_actuator_status(const antidrone_turret::msg::ActuatorStatus::SharedPtr msg)
  {
    last_actuator_ready_ = (msg->state == antidrone_turret::msg::ActuatorStatus::READY);
    last_trigger_count_ = msg->trigger_count;

    if (last_actuator_ready_) {
      RCLCPP_INFO(get_logger(), "actuator state: READY (trigger_count=%u)",
                  last_trigger_count_);
    } else {
      RCLCPP_WARN(get_logger(), "actuator state: RELOADING (trigger_count=%u)",
                  last_trigger_count_);
    }
  }

  void call_trigger_service(float confidence, float distance_m)
  {
    if (!trigger_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "trigger service not available");
      return;
    }

    auto request = std::make_shared<antidrone_turret::srv::TriggerActuator::Request>();
    request->confidence = confidence;
    request->distance_m = distance_m;

    auto result_callback = [this](rclcpp::Client<antidrone_turret::srv::TriggerActuator>::SharedFuture future) {
      if (future.get()) {
        if (future.get()->accepted) {
          RCLCPP_INFO(get_logger(), "trigger accepted, trigger_count=%u",
                      future.get()->trigger_count);
        } else {
          RCLCPP_WARN(get_logger(), "trigger rejected");
        }
      } else {
        RCLCPP_ERROR(get_logger(), "trigger service call failed");
      }
    };

    trigger_client_->async_send_request(request, result_callback);
  }

  double confidence_threshold_;
  double max_distance_m_;
  antidrone_turret::ControllerConfig config_;

  bool last_actuator_ready_{true};
  uint32_t last_trigger_count_{0};

  rclcpp::Publisher<antidrone_turret::msg::GimbalCommand>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<antidrone_turret::msg::ServoCommand>::SharedPtr servo_pub_;
  rclcpp::Publisher<antidrone_turret::msg::TurretStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<antidrone_turret::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<antidrone_turret::msg::ActuatorStatus>::SharedPtr actuator_sub_;
  rclcpp::Client<antidrone_turret::srv::TriggerActuator>::SharedPtr trigger_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}
