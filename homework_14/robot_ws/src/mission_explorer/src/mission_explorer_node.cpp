#include <chrono>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/srv/payload_trigger.hpp"
#include "underground_world/msg/robot_metrics.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"

#include "mission_explorer/explorer.hpp"

using namespace std::chrono_literals;

class MissionExplorerNode : public rclcpp::Node
{
public:
    MissionExplorerNode()
        : Node("mission_explorer")
    {
        // Create service client for /payload/trigger.
        trigger_client_ = this->create_client<underground_world::srv::PayloadTrigger>(
            "/payload/trigger");

        // Wait for service to be available.
        RCLCPP_INFO(get_logger(), "Waiting for /payload/trigger service...");
        if (!trigger_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(get_logger(), "/payload/trigger service not available!");
        }
        RCLCPP_INFO(get_logger(), "/payload/trigger service available.");

        // Create publishers.
        cmd_move_pub_ = create_publisher<underground_world::msg::MoveCommand>(
            "/robot/cmd_move", 10);
        status_pub_ = create_publisher<underground_world::msg::StudentStatus>(
            "/student/status", 10);

        // Create subscription for local scan.
        local_scan_sub_ = create_subscription<underground_world::msg::LocalScan>(
            "/robot/local_scan", 10,
            [this](const underground_world::msg::LocalScan::SharedPtr msg) {
                onLocalScan(msg);
            });

        // Create subscription for enemy_down confirmations.
        enemy_down_sub_ = create_subscription<underground_world::msg::EnemyDown>(
            "/payload/enemy_down", 10,
            [this](const underground_world::msg::EnemyDown::SharedPtr msg) {
                onEnemyDown(msg);
            });

        // Create subscription for robot_result to detect mission completion.
        result_sub_ = create_subscription<underground_world::msg::RobotResult>(
            "/robot/result", 10,
            [this](const underground_world::msg::RobotResult::SharedPtr msg) {
                onResult(msg);
            });

        // Timer for periodic status publishing.
        status_timer_ = create_wall_timer(
            200ms, [this]() { publishStatus(); });

        RCLCPP_INFO(get_logger(), "MissionExplorerNode started.");
    }

private:
    void onLocalScan(const underground_world::msg::LocalScan::SharedPtr msg)
    {
        current_scan_ = *msg;

        // Update internal map from scan.
        explorer_.updateFromScan(*msg);

        // Mark the current robot position as visited.
        explorer_.markVisited({msg->robot_x, msg->robot_y});

        // Initialize explorer with start position if not yet initialized.
        if (!explorer_initialized_) {
            // Try to find start from scan cells.
            for (const auto& cell : msg->cells) {
                if (cell.cell_type == "S") {
                    explorer_.initialize({cell.x, cell.y});
                    explorer_initialized_ = true;
                    RCLCPP_INFO(get_logger(),
                                "Explorer initialized with start at (%d,%d)",
                                cell.x, cell.y);
                    break;
                }
            }
            // If no S in scan, use robot position.
            if (!explorer_initialized_) {
                explorer_.initialize({msg->robot_x, msg->robot_y});
                explorer_initialized_ = true;
                RCLCPP_INFO(get_logger(),
                            "Explorer initialized with robot pos (%d,%d)",
                            msg->robot_x, msg->robot_y);
            }
        }

        // Update total passable count from metrics if available.
        if (current_metrics_.has_value()) {
            float coverage = current_metrics_->map_coverage_percent;
            int unique_seen = current_metrics_->unique_cells_seen;
            if (coverage > 0.0f && unique_seen > 0) {
                // Estimate total passable: unique_seen / (coverage / 100)
                int estimated_total = static_cast<int>(unique_seen * 100.0f / coverage + 0.5f);
                explorer_.setTotalPassableCount(estimated_total);
            }
        }

        // Check for visible contacts and trigger them.
        bool triggered = tryTriggerContacts(msg.get());

        if (!triggered) {
            makeDecision();
        }
    }

    void onEnemyDown(const underground_world::msg::EnemyDown::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(),
                    "enemy_down received: contact_id=%d at (%d,%d)",
                    msg->contact_id, msg->x, msg->y);
        explorer_.onEnemyDown(msg->contact_id, {msg->x, msg->y});

        // Clear pending contact.
        pending_contacts_.erase(msg->contact_id);

        // Publish updated status immediately.
        publishStatus();
    }

    void onResult(const underground_world::msg::RobotResult::SharedPtr msg)
    {
        current_result_ = *msg;

        if (msg->mission_result == "SUCCESS") {
            RCLCPP_INFO(get_logger(), "Mission SUCCESS! Reason: %s", msg->reason.c_str());
            current_state_ = underground_world::msg::StudentStatus::DONE;
            publishStatus();
        } else if (msg->mission_result == "FAILED_MAX_STEPS") {
            RCLCPP_WARN(get_logger(), "Mission FAILED! Reason: %s", msg->reason.c_str());
            current_state_ = underground_world::msg::StudentStatus::FAILED;
            publishStatus();
        }
    }

    bool tryTriggerContacts(const underground_world::msg::LocalScan* scan)
    {
        if (!scan) {
            return false;
        }

        // Check if mission is already complete - don't trigger more contacts.
        if (current_result_.mission_result == "SUCCESS" || current_result_.mission_result == "FAILED_MAX_STEPS") {
            return false;
        }

        bool triggered = false;
        for (const auto& cell : scan->cells) {
            if (cell.cell_type == "C") {
                // Check if already pending or processed.
                if (pending_contacts_.count(cell.contact_id) > 0) {
                    continue;
                }
                if (explorer_.wasContactProcessed(cell.contact_id)) {
                    continue;
                }

                RCLCPP_INFO(get_logger(),
                            "Triggering contact %d at (%d,%d)",
                            cell.contact_id, cell.x, cell.y);

                // Call /payload/trigger service.
                auto request = std::make_shared<underground_world::srv::PayloadTrigger::Request>();
                request->contact_id = cell.contact_id;
                request->x = cell.x;
                request->y = cell.y;

                pending_contacts_.insert(cell.contact_id);

                // Capture `this` (raw pointer) since the lambda runs in async callback.
                // Also capture request coords to avoid capturing the local `request` variable.
                int req_x = cell.x;
                int req_y = cell.y;
                trigger_client_->async_send_request(
                    request,
                    [self = this, cid = cell.contact_id, req_x, req_y](rclcpp::Client<underground_world::srv::PayloadTrigger>::SharedFuture future) {
                        auto response = future.get();
                        RCLCPP_INFO(self->get_logger(),
                                    "Trigger response for contact %d: accepted=%s reason=%s",
                                    cid,
                                    response->accepted ? "true" : "false",
                                    response->reason.c_str());
                        if (response->accepted) {
                            self->explorer_.onEnemyDown(cid, underground_world::Position{req_x, req_y});
                            self->pending_contacts_.erase(cid);
                        } else {
                            // Clear pending on failure to prevent deadlock.
                            self->pending_contacts_.erase(cid);
                            // Force state back to EXPLORING so next decide() issues a move.
                            self->current_state_ = underground_world::msg::StudentStatus::EXPLORING;
                        }
                    });

                triggered = true;
            }
        }

        if (triggered) {
            current_state_ = underground_world::msg::StudentStatus::ENGAGING;
            publishStatus();
        }

        return triggered;
    }

    bool shouldTriggerContact(const underground_world::msg::LocalScan* scan, int contact_id)
    {
        if (!scan) return false;
        // Check if already pending or processed.
        if (pending_contacts_.count(contact_id) > 0) {
            return false;
        }
        // Check if already processed by explorer.
        if (explorer_.wasContactProcessed(contact_id)) {
            return false;
        }
        return true;
    }

    void makeDecision()
    {
        // Check if mission is already complete.
        bool mission_done = (current_result_.mission_result == "SUCCESS" || 
                             current_result_.mission_result == "FAILED_MAX_STEPS");
        if (mission_done) {
            RCLCPP_INFO(get_logger(), "Mission already finished, not issuing more moves.");
            return;
        }

        if (!current_scan_.has_value()) {
            return;
        }

        // Update total passable count from metrics.
        if (current_metrics_.has_value()) {
            float coverage = current_metrics_->map_coverage_percent;
            int unique_seen = current_metrics_->unique_cells_seen;
            if (coverage > 0.0f && unique_seen > 0) {
                // Estimate total passable: unique_seen / (coverage / 100)
                int estimated_total = static_cast<int>(unique_seen * 100.0f / coverage + 0.5f);
                explorer_.setTotalPassableCount(estimated_total);
            }
        }

        // Get decision from explorer.
        auto decision = explorer_.decide();

        // Update our state from the decision.
        current_state_ = static_cast<uint8_t>(decision.state);

        RCLCPP_INFO(get_logger(),
                    "Decision: state=%u move=(%d,%d) status=%s",
                    static_cast<unsigned>(decision.state),
                    decision.move.has_value() ? decision.move->x : 0,
                    decision.move.has_value() ? decision.move->y : 0,
                    decision.status_text.c_str());

        // If there's a move command, convert position offset to direction enum.
        if (decision.move.has_value()) {
            underground_world::msg::MoveCommand cmd;
            int dx = decision.move->x;
            int dy = decision.move->y;

            if (dx == 0 && dy == -1) {
                cmd.direction = underground_world::msg::MoveCommand::UP;
            } else if (dx == 0 && dy == 1) {
                cmd.direction = underground_world::msg::MoveCommand::DOWN;
            } else if (dx == -1 && dy == 0) {
                cmd.direction = underground_world::msg::MoveCommand::LEFT;
            } else if (dx == 1 && dy == 0) {
                cmd.direction = underground_world::msg::MoveCommand::RIGHT;
            } else {
                RCLCPP_WARN(get_logger(), "Invalid move direction (%d,%d), skipping", dx, dy);
                return;
            }

            RCLCPP_INFO(get_logger(),
                        "Publishing move: direction=%u from (%d,%d)",
                        static_cast<unsigned>(cmd.direction),
                        current_scan_->robot_x,
                        current_scan_->robot_y);
            cmd_move_pub_->publish(cmd);
        }

        // Publish status.
        publishStatus();
    }

    void publishStatus()
    {
        underground_world::msg::StudentStatus msg;
        msg.state = current_state_;
        status_pub_->publish(msg);
    }

    // ROS interfaces.
    rclcpp::Subscription<underground_world::msg::LocalScan>::SharedPtr local_scan_sub_;
    rclcpp::Subscription<underground_world::msg::EnemyDown>::SharedPtr enemy_down_sub_;
    rclcpp::Subscription<underground_world::msg::RobotResult>::SharedPtr result_sub_;
    rclcpp::Publisher<underground_world::msg::MoveCommand>::SharedPtr cmd_move_pub_;
    rclcpp::Publisher<underground_world::msg::StudentStatus>::SharedPtr status_pub_;
    rclcpp::Client<underground_world::srv::PayloadTrigger>::SharedPtr trigger_client_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Core components.
    mission_explorer::Explorer explorer_;
    bool explorer_initialized_ = false;

    // State tracking.
    uint8_t current_state_ = underground_world::msg::StudentStatus::EXPLORING;
    std::set<int> pending_contacts_;

    // Latest observations.
    std::optional<underground_world::msg::LocalScan> current_scan_;
    std::optional<underground_world::msg::RobotMetrics> current_metrics_;
    underground_world::msg::RobotResult current_result_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionExplorerNode>());
    rclcpp::shutdown();
    return 0;
}
