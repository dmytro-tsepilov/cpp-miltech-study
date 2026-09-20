#pragma once

#include <string>
#include <memory>
#include <mutex>
#include <cmath>
#include <ctime>

#include <geometry_msgs/msg/twist.hpp>

namespace teleop_operator {

/// Teleoperation driver supporting keyboard and gamepad input
class TeleopDriver {
public:
    enum class InputType { KEYBOARD, JOY, NONE };
    
    TeleopDriver();
    ~TeleopDriver() = default;
    
    /// Initialize with given input type
    void init(InputType input_type = InputType::KEYBOARD);
    
    /// Get current command (linear_x, linear_y, angular_z)
    geometry_msgs::msg::Twist getCommand() const;
    
    /// Check if operator is active (any input received recently)
    bool isOperatorActive() const;
    
    /// Get time since last input (seconds)
    double getTimeSinceLastInput() const;
    
    /// Set deadzone for analog inputs (default 0.1)
    void setDeadzone(double deadzone);
    
    /// Set max linear speed (m/s, default 1.0)
    void setMaxLinearSpeed(double max_linear);
    
    /// Set max angular speed (rad/s, default 1.0)
    void setMaxAngularSpeed(double max_angular);
    
    /// Reset command to zero (called when switching away from teleop)
    void resetCommand();

private:
    // Apply deadzone filter
    void applyDeadzone(double& value) const;
    
    // Scale input with deadzone and limits
    double scaleInput(double raw, double max_output) const;
    
    // Input configuration
    InputType input_type_;
    double deadzone_ = 0.1;
    double max_linear_ = 1.0;   // m/s
    double max_angular_ = 1.0;  // rad/s
    
    // Current command state
    mutable std::mutex cmd_mutex_;
    geometry_msgs::msg::Twist current_cmd_;
    
    // Activity tracking
    std::time_t last_input_time_ = 0;
    bool operator_active_ = false;
};

} // namespace teleop_operator
