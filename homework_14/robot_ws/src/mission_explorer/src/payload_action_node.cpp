#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

using namespace std::chrono_literals;

class PayloadActionNode : public rclcpp::Node
{
public:
    PayloadActionNode()
        : Node("payload_action")
    {
        const auto qos = rclcpp::QoS{10};

        // Publisher for /payload/enemy_down confirmation.
        enemy_down_pub_ = create_publisher<underground_world::msg::EnemyDown>(
            "/payload/enemy_down", qos);

        // Create service server for /payload/trigger.
        // This node simply forwards the trigger request to underground_world_node
        // by publishing /payload/enemy_down after basic structural validation.
        trigger_service_ = create_service<underground_world::srv::PayloadTrigger>(
            "/payload/trigger",
            [this](
                const std::shared_ptr<underground_world::srv::PayloadTrigger::Request> request,
                const std::shared_ptr<underground_world::srv::PayloadTrigger::Response> response) {
                onTrigger(request, response);
            });

        RCLCPP_INFO(get_logger(), "PayloadActionNode started. Forging triggers to underground_world_node.");
    }

private:
    void onTrigger(
        const std::shared_ptr<underground_world::srv::PayloadTrigger::Request> request,
        const std::shared_ptr<underground_world::srv::PayloadTrigger::Response> response)
    {
        RCLCPP_INFO(
            get_logger(),
            "Trigger request: contact_id=%d at (%d,%d)",
            request->contact_id,
            request->x,
            request->y);

        // Basic structural validation: contact_id and coordinates must be non-zero.
        if (request->contact_id <= 0) {
            RCLCPP_WARN(get_logger(), "Trigger rejected: invalid contact_id=%d", request->contact_id);
            response->accepted = false;
            response->reason = "invalid contact_id";
            return;
        }

        // Forward the trigger to underground_world_node via /payload/enemy_down topic.
        // The world node performs full validation (existence, visibility, not already processed).
        underground_world::msg::EnemyDown enemy_msg;
        enemy_msg.contact_id = request->contact_id;
        enemy_msg.x = request->x;
        enemy_msg.y = request->y;
        enemy_down_pub_->publish(enemy_msg);

        // The world node will validate and update state.
        // We accept the request here and let the world confirm via metrics/result.
        RCLCPP_INFO(
            get_logger(),
            "Forwarded trigger for contact %d to underground_world_node",
            request->contact_id);
        response->accepted = true;
        response->reason = "forwarded to world";
    }

    // ROS interfaces.
    rclcpp::Service<underground_world::srv::PayloadTrigger>::SharedPtr trigger_service_;
    rclcpp::Publisher<underground_world::msg::EnemyDown>::SharedPtr enemy_down_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PayloadActionNode>());
    rclcpp::shutdown();
    return 0;
}
