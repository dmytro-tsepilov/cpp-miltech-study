#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "underground_world/scenario.hpp"
#include "underground_world/msg/local_scan.hpp"

namespace mission_explorer {

/// Exploration algorithm state.
enum class ExplorerState : uint8_t {
    EXPLORING = 0,
    ENGAGING = 1,
    RETURNING = 2,
    DONE = 3,
    FAILED = 4
};

constexpr uint8_t state_to_uint8(ExplorerState s) { return static_cast<uint8_t>(s); }

/// Contact trigger request.
struct ContactTrigger {
    int contact_id = 0;
    int x = 0;
    int y = 0;
};

/// Result of a decision step.
struct Decision {
    std::optional<underground_world::Position> move; // direction as position offset
    std::vector<ContactTrigger> contacts_to_trigger;
    ExplorerState state = ExplorerState::EXPLORING;
    std::string status_text; // brief description
};

class Explorer {
public:
    Explorer() = default;

    /// Initialize the explorer with a start position.
    void initialize(underground_world::Position start);

    /// Update current robot position (from scan.robot_x/robot_y).
    void updateRobotPosition(underground_world::Position pos);

    /// Update internal map from a LocalScan.
    void updateFromScan(const underground_world::msg::LocalScan& scan);

    /// Mark the current robot position as visited.
    void markVisited(underground_world::Position pos);

    /// Make a decision based on current observation and metrics.
    Decision decide();

    /// Called when an enemy_down confirmation is received for a contact.
    void onEnemyDown(int contact_id, underground_world::Position pos);

    /// Clear pending contacts on trigger failure (prevents deadlock).
    void clearPendingContacts();

    /// Get current state as uint8 for StudentStatus message.
    uint8_t getCurrentStateUint8() const { return state_to_uint8(current_state_); }

    /// Check if all passable cells are explored (for mission complete).
    bool isMissionComplete() const;

    /// Set total passable count (from world metrics).
    void setTotalPassableCount(int count) { total_passable_count_ = count; total_passable_computed_ = true; }

    /// Check if a contact was already processed.
    bool wasContactProcessed(int contact_id) const;

private:
    ExplorerState current_state_ = ExplorerState::EXPLORING;

    // Known map state.
    struct CellInfo {
        bool is_wall = false;
        bool is_start = false;
        bool is_contact_active = false;
        bool is_contact_processed = false;
        bool visited = false;
    };
    std::map<underground_world::Position, CellInfo> known_map_{};

    underground_world::Position start_position_{0, 0};
    underground_world::Position robot_position_{0, 0};

    // Contacts tracking.
    std::set<int> pending_contacts_;
    std::set<int> processed_contacts_;

    // Seen passable cells count.
    int seen_passable_count_ = 0;
    bool total_passable_computed_ = false;
    int total_passable_count_ = 0;

    // Direction helpers.
    static constexpr int DX[4] = {0, 0, -1, 1};  // UP, DOWN, LEFT, RIGHT
    static constexpr int DY[4] = {-1, 1, 0, 0};

    /// Update seen passable count from known_map_.
    void updateSeenCount();

    /// Count total passable cells via flood fill.
    int countPassableCells() const;

    /// Find unvisited reachable passable cells from current robot position.
    std::vector<underground_world::Position> findUnvisitedReachable() const;

    /// BFS pathfinding from current position to target.
    std::optional<std::vector<underground_world::Position>> bfsPath(
        underground_world::Position from, underground_world::Position to) const;

    /// Find direction from pos1 to nearest cell in targets.
    std::optional<underground_world::Position> findDirectionToTargets(
        const std::vector<underground_world::Position>& targets) const;

    /// Find nearest visited cell with unvisited passable neighbors.
    std::optional<underground_world::Position> findNearestUnvisitedFrontier() const;

    /// Check if all contacts are processed.
    bool allContactsProcessed() const;

    /// Check if all passable cells have been seen.
    bool allPassableSeen() const;
};

} // namespace mission_explorer
