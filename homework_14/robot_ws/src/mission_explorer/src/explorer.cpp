#include "mission_explorer/explorer.hpp"

#include <algorithm>
#include <queue>
#include <sstream>
#include <stack>

namespace mission_explorer {

void Explorer::initialize(underground_world::Position start)
{
    start_position_ = start;
    robot_position_ = start;
    current_state_ = ExplorerState::EXPLORING;
}

void Explorer::updateRobotPosition(underground_world::Position pos)
{
    robot_position_ = pos;
}

void Explorer::updateFromScan(const underground_world::msg::LocalScan& scan)
{
    for (const auto& cell : scan.cells) {
        underground_world::Position pos{cell.x, cell.y};
        CellInfo info;

        if (cell.cell_type == "#") {
            info.is_wall = true;
        } else if (cell.cell_type == ".") {
            info.is_wall = false;
        } else if (cell.cell_type == "S") {
            info.is_start = true;
            start_position_ = pos;
        } else if (cell.cell_type == "C") {
            info.is_contact_active = true;
        } else if (cell.cell_type == "x") {
            info.is_contact_processed = true;
        }

        auto it = known_map_.find(pos);
        if (it == known_map_.end()) {
            known_map_[pos] = info;
        } else {
            // Do not overwrite WALL with FREE.
            if (!info.is_wall || !it->second.is_wall) {
                if (info.is_start) it->second.is_start = true;
                if (info.is_contact_active) it->second.is_contact_active = true;
                if (info.is_contact_processed) it->second.is_contact_processed = true;
            }
        }
    }

    // Update robot position from scan.
    robot_position_ = {scan.robot_x, scan.robot_y};
}

void Explorer::markVisited(underground_world::Position pos)
{
    auto it = known_map_.find(pos);
    if (it != known_map_.end()) {
        it->second.visited = true;
    }
}

void Explorer::onEnemyDown(int contact_id, underground_world::Position pos)
{
    processed_contacts_.insert(contact_id);
    pending_contacts_.erase(contact_id);

    // Mark the cell as processed in the map.
    auto it = known_map_.find(pos);
    if (it != known_map_.end()) {
        it->second.is_contact_active = false;
        it->second.is_contact_processed = true;
    }
}

void Explorer::clearPendingContacts()
{
    pending_contacts_.clear();
}

void Explorer::updateSeenCount()
{
    seen_passable_count_ = 0;
    for (const auto& [pos, info] : known_map_) {
        if (!info.is_wall) {
            ++seen_passable_count_;
        }
    }
}

int Explorer::countPassableCells() const
{
    int count = 0;
    std::set<underground_world::Position> visited;
    std::queue<underground_world::Position> q;

    q.push(start_position_);
    visited.insert(start_position_);

    while (!q.empty()) {
        auto current = q.front();
        q.pop();
        ++count;

        for (int d = 0; d < 4; ++d) {
            underground_world::Position neighbor{current.x + DX[d], current.y + DY[d]};
            if (visited.count(neighbor) > 0) continue;

            auto it = known_map_.find(neighbor);
            if (it == known_map_.end()) continue;
            if (it->second.is_wall) continue;

            visited.insert(neighbor);
            q.push(neighbor);
        }
    }

    return count;
}

std::vector<underground_world::Position> Explorer::findUnvisitedReachable() const
{
    std::vector<underground_world::Position> result;
    std::set<underground_world::Position> visited;
    std::queue<underground_world::Position> q;

    q.push(robot_position_);
    visited.insert(robot_position_);

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        for (int d = 0; d < 4; ++d) {
            underground_world::Position neighbor{current.x + DX[d], current.y + DY[d]};
            if (visited.count(neighbor) > 0) continue;

            auto it = known_map_.find(neighbor);
            if (it == known_map_.end()) continue;
            if (it->second.is_wall) continue;

            visited.insert(neighbor);
            // Check if unvisited.
            bool is_visited = it->second.visited;
            if (!is_visited) {
                result.push_back(neighbor);
            }
            q.push(neighbor);
        }
    }

    return result;
}

std::optional<underground_world::Position> Explorer::findNearestUnvisitedFrontier() const
{
    // Find all visited cells that have unexplored neighbors (either unknown or unvisited passable).
    // Return the one closest to robot_position_ via BFS.
    struct Candidate {
        underground_world::Position pos;
        int dist = INT32_MAX;
    };

    std::vector<Candidate> candidates;

    for (const auto& [pos, info] : known_map_) {
        if (info.is_wall || !info.visited) continue;

        // Check if this visited cell has any unexplored neighbors.
        for (int d = 0; d < 4; ++d) {
            underground_world::Position neighbor{pos.x + DX[d], pos.y + DY[d]};
            auto it = known_map_.find(neighbor);
            if (it == known_map_.end()) {
                // Neighbor not yet known - this is a true frontier!
                int dist = std::abs(neighbor.x - robot_position_.x) +
                           std::abs(neighbor.y - robot_position_.y);
                candidates.push_back({pos, dist});
                break;
            }
            if (it->second.is_wall) continue;
            if (it->second.visited) continue;

            int dist = std::abs(neighbor.x - robot_position_.x) +
                       std::abs(neighbor.y - robot_position_.y);
            candidates.push_back({pos, dist});
            break;
        }
    }

    if (candidates.empty()) return std::nullopt;

    // Find candidate closest to robot via BFS.
    std::map<underground_world::Position, int> dist_map;
    std::set<underground_world::Position> visited;
    std::queue<underground_world::Position> q;

    q.push(robot_position_);
    visited.insert(robot_position_);
    dist_map[robot_position_] = 0;

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        for (int d = 0; d < 4; ++d) {
            underground_world::Position neighbor{current.x + DX[d], current.y + DY[d]};
            if (visited.count(neighbor) > 0) continue;
            auto it = known_map_.find(neighbor);
            if (it == known_map_.end()) continue;
            if (it->second.is_wall) continue;

            visited.insert(neighbor);
            dist_map[neighbor] = dist_map[current] + 1;
            q.push(neighbor);
        }
    }

    underground_world::Position best_pos{};
    int best_bfs_dist = INT32_MAX;

    for (const auto& cand : candidates) {
        auto dit = dist_map.find(cand.pos);
        if (dit != dist_map.end() && dit->second < best_bfs_dist) {
            best_bfs_dist = dit->second;
            best_pos = cand.pos;
        }
    }

    return best_bfs_dist < INT32_MAX ? std::optional<underground_world::Position>(best_pos) : std::nullopt;
}

std::optional<std::vector<underground_world::Position>> Explorer::bfsPath(
    underground_world::Position from, underground_world::Position to) const
{
    if (from == to) {
        return std::vector<underground_world::Position>{};
    }

    std::map<underground_world::Position, underground_world::Position> parent;
    std::set<underground_world::Position> visited;
    std::queue<underground_world::Position> q;

    q.push(from);
    visited.insert(from);

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        for (int d = 0; d < 4; ++d) {
            underground_world::Position neighbor{current.x + DX[d], current.y + DY[d]};
            if (visited.count(neighbor) > 0) continue;

            auto it = known_map_.find(neighbor);
            if (it == known_map_.end()) continue;
            if (it->second.is_wall) continue;

            parent[neighbor] = current;
            visited.insert(neighbor);

            if (neighbor == to) {
                std::vector<underground_world::Position> path;
                underground_world::Position cur = to;
                while (cur != from) {
                    path.push_back(cur);
                    auto pit = parent.find(cur);
                    if (pit == parent.end()) return std::nullopt;
                    cur = pit->second;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            q.push(neighbor);
        }
    }

    return std::nullopt;
}

std::optional<underground_world::Position> Explorer::findDirectionToTargets(
    const std::vector<underground_world::Position>& targets) const
{
    if (targets.empty()) return std::nullopt;

    std::set<underground_world::Position> target_set(targets.begin(), targets.end());

    // Check if any immediate neighbor IS a target.
    for (int d = 0; d < 4; ++d) {
        underground_world::Position neighbor{robot_position_.x + DX[d], robot_position_.y + DY[d]};
        if (target_set.find(neighbor) != target_set.end()) {
            return underground_world::Position{DX[d], DY[d]};
        }
    }

    // No immediate neighbor is a target. Find nearest via BFS.
    std::map<underground_world::Position, underground_world::Position> parent;
    std::set<underground_world::Position> visited;
    std::queue<underground_world::Position> q;

    q.push(robot_position_);
    visited.insert(robot_position_);

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        for (int d = 0; d < 4; ++d) {
            underground_world::Position neighbor{current.x + DX[d], current.y + DY[d]};
            if (visited.count(neighbor) > 0) continue;

            auto it = known_map_.find(neighbor);
            if (it == known_map_.end()) continue;
            if (it->second.is_wall) continue;

            parent[neighbor] = current;
            visited.insert(neighbor);

            if (target_set.find(neighbor) != target_set.end()) {
                // Trace back from neighbor to find first step toward it.
                underground_world::Position cur = neighbor;
                while (true) {
                    auto pit = parent.find(cur);
                    if (pit == parent.end()) return std::nullopt;
                    if (pit->second == robot_position_) {
                        // cur is adjacent to robot - this is the first step.
                        return underground_world::Position{cur.x - robot_position_.x, cur.y - robot_position_.y};
                    }
                    cur = pit->second;
                }
            }

            q.push(neighbor);
        }
    }

    return std::nullopt;
}

bool Explorer::wasContactProcessed(int contact_id) const
{
    return processed_contacts_.count(contact_id) > 0;
}

bool Explorer::allContactsProcessed() const
{
    size_t total_contacts = pending_contacts_.size() + processed_contacts_.size();
    return total_contacts > 0 && pending_contacts_.empty();
}

bool Explorer::allPassableSeen() const
{
    auto unvisited = findUnvisitedReachable();
    return unvisited.empty();
}

Decision Explorer::decide()
{
    Decision decision;

    updateSeenCount();

    // Check for visible contacts.
    if (!pending_contacts_.empty()) {
        current_state_ = ExplorerState::ENGAGING;
        decision.state = ExplorerState::ENGAGING;
        decision.status_text = "Engaging contacts";
        return decision;
    }

    if (current_state_ == ExplorerState::ENGAGING) {
        current_state_ = ExplorerState::EXPLORING;
        decision.state = ExplorerState::EXPLORING;
    }

    // Check mission complete conditions.
    if (allContactsProcessed() && allPassableSeen()) {
        current_state_ = ExplorerState::DONE;
        decision.state = ExplorerState::DONE;
        decision.status_text = "Mission complete";
        return decision;
    }

    // Exploration phase: find next unvisited reachable cell.
    auto unvisited = findUnvisitedReachable();

    if (!unvisited.empty()) {
        current_state_ = ExplorerState::EXPLORING;
        decision.state = ExplorerState::EXPLORING;

        auto dir = findDirectionToTargets(unvisited);
        if (dir.has_value()) {
            decision.move = dir.value();
            decision.status_text = "Exploring: moving to unvisited cell";
        }
    } else {
        // All known reachable cells are visited. Check for true frontiers.
        auto frontier = findNearestUnvisitedFrontier();
        if (frontier.has_value()) {
            // Navigate to the frontier cell.
            auto path = bfsPath(robot_position_, frontier.value());
            if (path.has_value() && !path->empty()) {
                decision.move = underground_world::Position{
                    (*path)[0].x - robot_position_.x,
                    (*path)[0].y - robot_position_.y};
                current_state_ = ExplorerState::EXPLORING;
                decision.state = ExplorerState::EXPLORING;
                decision.status_text = "Repositioning to find unvisited cells";
            } else {
                // Can't reach frontier, return to start.
                auto path_to_start = bfsPath(robot_position_, start_position_);
                if (path_to_start.has_value() && !path_to_start->empty()) {
                    decision.move = underground_world::Position{
                        (*path_to_start)[0].x - robot_position_.x,
                        (*path_to_start)[0].y - robot_position_.y};
                    current_state_ = ExplorerState::EXPLORING;
                    decision.state = ExplorerState::EXPLORING;
                    decision.status_text = "Returning to start (no path to frontier)";
                } else {
                    current_state_ = ExplorerState::FAILED;
                    decision.state = ExplorerState::FAILED;
                    decision.status_text = "Stuck: no path to start";
                }
            }
        } else {
            // No valid recovery target - check if we really are done.
            const int total = countPassableCells();
            if (total > 0 && seen_passable_count_ >= total && allContactsProcessed()) {
                current_state_ = ExplorerState::DONE;
                decision.state = ExplorerState::DONE;
                decision.status_text = "All cells explored";
                return decision;
            }
            // Return to start as fallback.
            auto path_to_start = bfsPath(robot_position_, start_position_);
            if (path_to_start.has_value() && !path_to_start->empty()) {
                decision.move = underground_world::Position{
                    (*path_to_start)[0].x - robot_position_.x,
                    (*path_to_start)[0].y - robot_position_.y};
                current_state_ = ExplorerState::EXPLORING;
                decision.state = ExplorerState::EXPLORING;
                decision.status_text = "Returning to start (no frontier)";
            } else {
                current_state_ = ExplorerState::FAILED;
                decision.state = ExplorerState::FAILED;
                decision.status_text = "Stuck: no path to start";
            }
        }
    }

    return decision;
}

bool Explorer::isMissionComplete() const
{
    return current_state_ == ExplorerState::DONE;
}

} // namespace mission_explorer
