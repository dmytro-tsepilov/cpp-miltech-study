#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <cstdio>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "teleop_operator/teleop_driver.hpp"

using namespace std::chrono_literals;

/// Teleop node - handles keyboard and gamepad input for robot control
class TeleopNode : public rclcpp::Node {
public:
    TeleopNode()
        : Node("teleop_node")
    {
        RCLCPP_INFO(get_logger(), "Teleop Node starting...");
        
        // Declare parameters
        input_type_ = declare_parameter("input_type", "keyboard");  // keyboard, joy
        max_linear_ = declare_parameter("max_linear", 1.0);         // m/s
        max_angular_ = declare_parameter("max_angular", 1.5);      // rad/s
        deadzone_ = declare_parameter("deadzone", 0.1);
        cmd_pub_rate_ = declare_parameter("cmd_pub_rate", 10);     // Hz
        
        // Initialize teleop driver
        teleop_driver_.init(parseInputType(input_type_));
        teleop_driver_.setMaxLinearSpeed(max_linear_);
        teleop_driver_.setMaxAngularSpeed(max_angular_);
        teleop_driver_.setDeadzone(deadzone_);
        
        // Publishers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/control/cmd_vel", 10);
        
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/teleop/status", 10);
        
        // Subscriptions (for gamepad input)
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                onJoyMessage(*msg);
            });
        
        // Command publisher timer
        cmd_pub_timer_ = create_wall_timer(
            100ms / cmd_pub_rate_,
            [this]() { publishCommand(); });
        
        // Keyboard input thread (non-blocking)
        key_thread_ = std::thread(&TeleopNode::keyboardInputLoop, this);
        
        RCLCPP_INFO(get_logger(), "Teleop Node initialized");
        RCLCPP_INFO(get_logger(), "Input type: %s, max_linear: %.2f, max_angular: %.2f",
                     input_type_.c_str(), max_linear_, max_angular_);
    }
    
    ~TeleopNode() override {
        running_ = false;
        if (key_thread_.joinable()) {
            key_thread_.join();
        }
    }

private:
    teleop_operator::TeleopDriver::InputType parseInputType(const std::string& type) {
        if (type == "joy") return teleop_operator::TeleopDriver::InputType::JOY;
        return teleop_operator::TeleopDriver::InputType::KEYBOARD;
    }
    
    // Gamepad input handler
    void onJoyMessage(const sensor_msgs::msg::Joy& msg) {
        if (msg.axes.size() < 6) return;
        
        // Standard gamepad mapping:
        // axis 1: left stick up/down -> linear.x (forward/backward)
        // axis 0: left stick left/right -> linear.y (strafe, if supported)
        // axis 3: right stick up/down -> angular.z (rotation)
        
        teleop_driver_.setMaxLinearSpeed(max_linear_);
        teleop_driver_.setMaxAngularSpeed(max_angular_);
    }
    
    // Publish command to /control/cmd_vel
    void publishCommand() {
        auto cmd = teleop_driver_.getCommand();
        
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = now();
        twist_msg.twist = cmd;
        
        cmd_vel_pub_->publish(twist_msg);
        
        // Publish status if operator is active
        if (teleop_driver_.isOperatorActive()) {
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "TELEOP ACTIVE: linear=" + 
                             std::to_string(cmd.linear.x) + 
                             ", angular=" + std::to_string(cmd.angular.z);
            status_pub_->publish(status_msg);
        }
    }
    
    // Keyboard input loop (runs in separate thread)
    void keyboardInputLoop() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        
        while (rclcpp::ok() && running_) {
            // Non-blocking keyboard input using termios
            int ch = getchar();
            
            if (ch == -1) {  // No input
                std::this_thread::sleep_for(10ms);
                continue;
            }
            
            bool input_received = false;
            geometry_msgs::msg::Twist cmd;
            
            switch (ch) {
                case 'w': case 'W': cmd.linear.x = max_linear_; input_received = true; break;
                case 's': case 'S': cmd.linear.x = -max_linear_; input_received = true; break;
                case 'a': case 'A': cmd.linear.y = -max_linear_ * 0.5; input_received = true; break;
                case 'd': case 'D': cmd.linear.y = max_linear_ * 0.5; input_received = true; break;
                case 'q': case 'Q': cmd.angular.z = max_angular_; input_received = true; break;
                case 'e': case 'E': cmd.angular.z = -max_angular_; input_received = true; break;
                case ' ':  // Space bar - stop all
                    cmd.linear.x = 0.0;
                    cmd.linear.y = 0.0;
                    cmd.angular.z = 0.0;
                    input_received = true;
                    RCLCPP_INFO(get_logger(), "STOP command received");
                    break;
                default:
                    continue;
            }
            
            if (input_received) {
                // Publish directly
                geometry_msgs::msg::TwistStamped twist_msg;
                twist_msg.header.stamp = now();
                twist_msg.twist.linear.x = cmd.linear.x;
                twist_msg.twist.linear.y = cmd.linear.y;
                twist_msg.twist.angular.z = cmd.angular.z;
                cmd_vel_pub_->publish(twist_msg);
                
                last_input_time_ = std::time(nullptr);
                operator_active_ = true;
            }
        }
        
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
    
    // Parameters
    std::string input_type_;
    double max_linear_;
    double max_angular_;
    double deadzone_;
    int cmd_pub_rate_;
    
    // Teleop driver
    teleop_operator::TeleopDriver teleop_driver_;
    
    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr cmd_pub_timer_;
    
    // State
    std::thread key_thread_;
    bool running_ = true;
    std::time_t last_input_time_;
    bool operator_active_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
