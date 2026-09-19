#include "teleop_operator/teleop_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace teleop_operator {

TeleopDriver::TeleopDriver() 
    : input_type_(InputType::KEYBOARD) {
    current_cmd_.linear.x = 0.0;
    current_cmd_.linear.y = 0.0;
    current_cmd_.angular.z = 0.0;
}

void TeleopDriver::init(InputType input_type) {
    input_type_ = input_type;
    resetCommand();
    
    RCLCPP_INFO(rclcpp::get_logger("teleop_driver"), 
                "Teleop driver initialized with input type: %d", static_cast<int>(input_type));
}

void TeleopDriver::applyDeadzone(double& value) const {
    if (std::abs(value) < deadzone_) {
        value = 0.0;
    }
}

double TeleopDriver::scaleInput(double raw, double max_output) const {
    applyDeadzone(raw);
    // Normalize to [-1, 1] range
    double normalized = std::clamp(raw / (1.0 + deadzone_), -1.0, 1.0);
    return normalized * max_output;
}

geometry_msgs::msg::Twist TeleopDriver::getCommand() const {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    return current_cmd_;
}

bool TeleopDriver::isOperatorActive() const {
    auto now = std::time(nullptr);
    double elapsed = std::difftime(now, last_input_time_);
    return operator_active_ && (elapsed < 5.0); // Consider active within 5 seconds
}

double TeleopDriver::getTimeSinceLastInput() const {
    auto now = std::time(nullptr);
    return std::difftime(now, last_input_time_);
}

void TeleopDriver::setDeadzone(double deadzone) {
    deadzone_ = std::max(0.0, std::min(1.0, deadzone));
}

void TeleopDriver::setMaxLinearSpeed(double max_linear) {
    max_linear_ = std::max(0.01, max_linear);
}

void TeleopDriver::setMaxAngularSpeed(double max_angular) {
    max_angular_ = std::max(0.01, max_angular);
}

void TeleopDriver::resetCommand() {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    current_cmd_.linear.x = 0.0;
    current_cmd_.linear.y = 0.0;
    current_cmd_.linear.z = 0.0;
    current_cmd_.angular.x = 0.0;
    current_cmd_.angular.y = 0.0;
    current_cmd_.angular.z = 0.0;
}

} // namespace teleop_operator
