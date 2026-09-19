// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

/// Simple waypoint structure
struct Waypoint2D {
    double x;
    double y;
};

/// Simple waypoint follower for fake odometry simulation
class FakeOdomPublisher : public rclcpp::Node {
public:
    FakeOdomPublisher()
        : Node("fake_odom_publisher")
        , current_waypoint_idx_(0)
        , mission_complete_(false)
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

        // Publishers
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/robot/pose", 10);
        
        // Publish robot position as string for mine_spawner
        if (use_robot_pos_topic_) {
            robot_pos_pub_ = create_publisher<std_msgs::msg::String>("/robot/position", 10);
        }

        // Timer for odometry publishing
        odom_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_ * 10)),
            [this]() { publishOdom(); });

        // Timer for waypoint checking
        wp_check_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { checkWaypoint(); });

        current_x_ = waypoints_[0].x;
        current_y_ = waypoints_[0].y;
        current_heading_ = 0.0;

        RCLCPP_INFO(get_logger(), "FakeOdomPublisher initialized");
    }

private:
    void publishOdom() {
        if (mission_complete_) return;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = current_x_;
        odom_msg.pose.pose.position.y = current_y_;
        odom_msg.pose.pose.orientation.z = std::sin(current_heading_ / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(current_heading_ / 2.0);

        // Calculate linear velocity towards next waypoint
        if (current_waypoint_idx_ < static_cast<int>(waypoints_.size())) {
            double dx = waypoints_[current_waypoint_idx_].x - current_x_;
            double dy = waypoints_[current_waypoint_idx_].y - current_y_;
            double dist = std::hypot(dx, dy);
            
            if (dist > 0.01) {
                odom_msg.twist.twist.linear.x = (dx / dist) * speed_;
                odom_msg.twist.twist.linear.y = (dy / dist) * speed_;
            } else {
                odom_msg.twist.twist.linear.x = 0.0;
                odom_msg.twist.twist.linear.y = 0.0;
            }
        }

        odom_pub_->publish(odom_msg);

        // Also publish pose
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = current_x_;
        pose_msg.pose.pose.position.y = current_y_;
        pose_msg.pose.pose.orientation.z = std::sin(current_heading_ / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(current_heading_ / 2.0);
        pose_pub_->publish(pose_msg);

        // Publish robot position as string for mine_spawner
        if (robot_pos_pub_) {
            std_msgs::msg::String pos_msg;
            pos_msg.data = std::to_string(current_x_) + "," + std::to_string(current_y_);
            robot_pos_pub_->publish(pos_msg);
        }
    }

    void checkWaypoint() {
        if (mission_complete_ || current_waypoint_idx_ >= static_cast<int>(waypoints_.size())) return;

        double dx = waypoints_[current_waypoint_idx_].x - current_x_;
        double dy = waypoints_[current_waypoint_idx_].y - current_y_;
        double dist = std::hypot(dx, dy);

        if (dist < 0.5) {
            RCLCPP_INFO(get_logger(), "Reached waypoint %d (%.1f, %.1f)",
                        current_waypoint_idx_ + 1,
                        waypoints_[current_waypoint_idx_].x,
                        waypoints_[current_waypoint_idx_].y);

            current_waypoint_idx_++;

            if (current_waypoint_idx_ >= static_cast<int>(waypoints_.size())) {
                // Check if closed loop - if so, restart from beginning
                RCLCPP_INFO(get_logger(), "All waypoints completed. Mission complete!");
                mission_complete_ = true;
                
                // After 5 seconds, restart for continuous patrol
                auto timer = create_wall_timer(
                    std::chrono::seconds(5),
                    [this]() {
                        RCLCPP_INFO(get_logger(), "Restarting patrol...");
                        current_waypoint_idx_ = 0;
                        mission_complete_ = false;
                        current_x_ = waypoints_[0].x;
                        current_y_ = waypoints_[0].y;
                    });
            } else {
                // Calculate heading to next waypoint
                double next_wp = std::atan2(
                    waypoints_[current_waypoint_idx_].y - current_y_,
                    waypoints_[current_waypoint_idx_].x - current_x_
                );
                current_heading_ = next_wp;
            }
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_pos_pub_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr wp_check_timer_;

    std::vector<Waypoint2D> waypoints_;
    double current_x_, current_y_;
    double current_heading_;
    int current_waypoint_idx_;
    double speed_;
    double publish_rate_hz_;
    bool use_robot_pos_topic_;
    bool mission_complete_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
