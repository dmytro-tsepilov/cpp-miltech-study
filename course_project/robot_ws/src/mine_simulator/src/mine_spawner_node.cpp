#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "perimeter_msgs/msg/mine_detection.hpp"
#include "mine_simulator/mine_config.hpp"

using namespace std::chrono_literals;

/// Mine spawner node - simulates mine detection for testing
class MineSpawnerNode : public rclcpp::Node {
public:
    MineSpawnerNode()
        : Node("mine_spawner")
        , config_(loadMineConfig())
        , rng_(std::random_device{}())
    {
        RCLCPP_INFO(get_logger(), "MineSpawnerNode starting...");
        RCLCPP_INFO(get_logger(), "Scenario: %s", config_.name.c_str());
        RCLCPP_INFO(get_logger(), "Mines: %zu", config_.mines.size());
        
        // Publisher for mine detections
        mine_pub_ = create_publisher<perimeter_msgs::msg::MineDetection>(
            "/mines/detected", 10);
        
        // Subscription for robot position (from odometry)
        robot_pos_sub_ = create_subscription<std_msgs::msg::String>(
            "/robot/position", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                onRobotPosition(msg);
            });
        
        // Timer for simulation update
        update_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(config_.update_period_ms)),
            [this]() { simulateDetection(); });
        
        RCLCPP_INFO(get_logger(), "MineSpawnerNode initialized");
    }

private:
    void onRobotPosition(const std_msgs::msg::String::SharedPtr msg) {
        // Parse simple "x,y" format
        auto parts = splitString(msg->data, ',');
        if (parts.size() >= 2) {
            try {
                robot_x_ = std::stod(parts[0]);
                robot_y_ = std::stod(parts[1]);
                robot_pos_valid_ = true;
            } catch (...) {
                // Ignore parse errors
            }
        }
    }
    
    void simulateDetection() {
        if (!robot_pos_valid_) {
            return; // Wait for first robot position
        }
        
        for (auto& mine : config_.mines) {
            if (mine.detected || mine.cleared) continue;
            
            // Compute distance to robot
            double dx = mine.x - robot_x_;
            double dy = mine.y - robot_y_;
            double dist = std::hypot(dx, dy);
            
            // Check if within detection range
            if (dist <= config_.detection_range) {
                // Probability decreases with distance
                double prob = config_.detection_probability * 
                              (1.0 - dist / config_.detection_range * config_.detection_decay);
                
                // Random check
                std::uniform_real_distribution<double> dist_fn(0.0, 1.0);
                if (dist_fn(rng_) < prob) {
                    // Mine detected!
                    auto detection = perimeter_msgs::msg::MineDetection();
                    detection.mine_id = mine.id;
                    detection.x = mine.x;
                    detection.y = mine.y;
                    detection.type = mine.type;
                    detection.confidence = static_cast<float>(prob);
                    
                    builtin_interfaces::msg::Time now;
                    now.sec = get_clock()->now().seconds();
                    now.nanosec = get_clock()->now().nanoseconds();
                    detection.detected_at = now;
                    
                    mine_pub_->publish(detection);
                    mine.detected = true;
                    
                    RCLCPP_INFO(get_logger(), "Mine detected! ID=%d at (%.1f, %.1f) type=%s confidence=%.2f",
                                mine.id, mine.x, mine.y, mine.type.c_str(), prob);
                }
            }
        }
    }
    
    // Utility: split string by delimiter
    std::vector<std::string> splitString(const std::string& s, char delim) {
        std::vector<std::string> parts;
        std::string current;
        for (char c : s) {
            if (c == delim) {
                if (!current.empty()) {
                    parts.push_back(current);
                    current.clear();
                }
            } else {
                current += c;
            }
        }
        if (!current.empty()) {
            parts.push_back(current);
        }
        return parts;
    }
    
    // Load default mine configuration
    mine_simulator::MineSimConfig loadMineConfig() {
        mine_simulator::MineSimConfig config;
        config.name = "training_ground";
        
        // Default training mines (on perimeter)
        config.mines.push_back({1, 10.0, 5.0, "anti-tank"});
        config.mines.push_back({2, 15.0, 15.0, "anti-personnel"});
        config.mines.push_back({3, 5.0, 10.0, "unknown"});
        
        config.detection_range = 3.0;
        config.detection_probability = 0.95;
        config.update_period_ms = 100.0;
        
        return config;
    }
    
    mine_simulator::MineSimConfig config_;
    std::mt19937 rng_;
    
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool robot_pos_valid_ = false;
    
    rclcpp::Publisher<perimeter_msgs::msg::MineDetection>::SharedPtr mine_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_pos_sub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MineSpawnerNode>());
    rclcpp::shutdown();
    return 0;
}
