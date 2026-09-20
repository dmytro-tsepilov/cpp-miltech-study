// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"

/// Simple waypoint structure
struct Waypoint2D {
    double x;
    double y;
};

/// Fake odometry publisher that integrates /control/cmd_vel commands
/// This simulates a robot base that responds to velocity commands
class FakeOdomPublisher : public rclcpp::Node {
public:
    FakeOdomPublisher()
        : Node("fake_odom_publisher")
        , current_waypoint_idx_(0)
        , mission_complete_(false)
        , linear_vel_(0.0)
        , angular_vel_(0.0)
    {
        RCLCPP_INFO(get_logger(), "FakeOdomPublisher starting...");

        // Declare parameters
        declare_parameter("waypoints_flat", std::vector<double>{0.0, 0.0, 20.0, 0.0, 20.0, 20.0, 0.0, 20.0});
        declare_parameter("speed", 2.0);
        declare_parameter("publish_rate_hz", 10.0);
        declare_parameter("use_robot_position_topic", false);

        // Get waypoints from parameter (flat list: x1, y1, x2, y2, ...)
        auto wp_flat = get_parameter("waypoints_flat").as_double_array();
        
        for (size_t i = 0; i + 1 < wp_flat.size(); i += 2) {
            waypoints_.push_back({wp_flat[i], wp_flat[i + 1]});
        }

        speed_ = get_parameter("speed").as_double();
        publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
        use_robot_pos_topic_ = get_parameter("use_robot_position_topic").as_bool();

        if (waypoints_.empty()) {
            // Default training ground perimeter
            waypoints_ = {
                {0.0, 0.0},
                {20.0, 0.0},
                {20.0, 20.0},
                {0.0, 20.0}
            };
        }

        RCLCPP_INFO(get_logger(), "Using %zu waypoints", waypoints_.size());
        RCLCPP_INFO(get_logger(), "Speed: %.1f m/s, Publish rate: %.1f Hz", speed_, publish_rate_hz_);

        // Subscribe to control commands - THIS IS THE KEY FIX!
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/control/cmd_vel", 10,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                onCmdVel(msg);
            });

        // Publishers
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        // Publish robot position as string for mine_spawner
        if (use_robot_pos_topic_) {
            robot_pos_pub_ = create_publisher<std_msgs::msg::String>("/robot/position", 10);
        }

        // Timer for odometry publishing
        odom_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_ * 10)),
            [this]() { publishOdom(); });

        current_x_ = waypoints_[0].x;
        current_y_ = waypoints_[0].y;
        current_heading_ = 0.0;

        RCLCPP_INFO(get_logger(), "FakeOdomPublisher initialized");
    }

private:
    void onCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        // Store the latest velocity command
        linear_vel_ = msg->twist.linear.x;
        angular_vel_ = msg->twist.angular.z;
    }

    void publishOdom() {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        // Integrate velocity to update position (Euler integration)
        double dt = 1.0 / publish_rate_hz_;
        
        // DIAGNOSTIC: Log heading and velocity state every 50 calls
        static int odom_count = 0;
        odom_count++;
        if (odom_count <= 10 || odom_count % 50 == 0) {
            RCLCPP_INFO(get_logger(),
                "[FAKE_ODOM] #%d: pos=(%.3f, %.3f), heading=%.4f rad (%.1f deg), vel=(%.2f, %.2f)",
                odom_count, current_x_, current_y_, current_heading_,
                current_heading_ * 180.0 / M_PI, linear_vel_, angular_vel_);
        }
        
        if (std::abs(angular_vel_) > 1e-6) {
            // Turn first
            current_heading_ += angular_vel_ * dt;
            
            // Then move forward
            current_x_ += linear_vel_ * std::cos(current_heading_) * dt;
            current_y_ += linear_vel_ * std::sin(current_heading_) * dt;
        } else {
            // Move straight
            current_x_ += linear_vel_ * std::cos(current_heading_) * dt;
            current_y_ += linear_vel_ * std::sin(current_heading_) * dt;
        }

        odom_msg.pose.pose.position.x = current_x_;
        odom_msg.pose.pose.position.y = current_y_;
        // FIX: Set proper quaternion for rotation around Z axis only
        // For pure yaw rotation: q.x=0, q.y=0, q.z=sin(heading/2), q.w=cos(heading/2)
        double q_z = std::sin(current_heading_ / 2.0);
        double q_w = std::cos(current_heading_ / 2.0);
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = q_z;
        odom_msg.pose.pose.orientation.w = q_w;

        // Publish the velocity we're using (for debugging)
        odom_msg.twist.twist.linear.x = linear_vel_;
        odom_msg.twist.twist.angular.z = angular_vel_;

        odom_pub_->publish(odom_msg);

        // Publish robot position as string for mine_spawner
        if (robot_pos_pub_) {
            std_msgs::msg::String pos_msg;
            pos_msg.data = std::to_string(current_x_) + "," + std::to_string(current_y_);
            robot_pos_pub_->publish(pos_msg);
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_pos_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    std::vector<Waypoint2D> waypoints_;
    double current_x_, current_y_;
    double current_heading_;
    double linear_vel_;
    double angular_vel_;
    double speed_;
    double publish_rate_hz_;
    bool use_robot_pos_topic_;
    bool mission_complete_;
    int current_waypoint_idx_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
