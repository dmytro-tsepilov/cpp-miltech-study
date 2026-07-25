#include "mission_explorer/map_builder.hpp"

#include <algorithm>
#include <queue>

namespace mission_explorer {

void MapBuilder::updateFromScan(const underground_world::msg::LocalScan& scan)
{
    // Update start position if not set.
    if (start_position_.x == 0 && start_position_.y == 0) {
        // We don't know the true start yet; rely on setStartPosition().
    }

    for (const auto& cell : scan.cells) {
        underground_world::Position pos{cell.x, cell.y};
        CellType type = CellType::UNKNOWN;

        if (cell.cell_type == "#") {
            type = CellType::WALL;
        } else if (cell.cell_type == ".") {
            type = CellType::FREE;
        } else if (cell.cell_type == "S") {
            type = CellType::START;
            start_position_ = pos;
        } else if (cell.cell_type == "C") {
            type = CellType::CONTACT_ACTIVE;
        } else if (cell.cell_type == "x") {
            type = CellType::CONTACT_PROCESSED;
        }

        // Only update if we don't have a more specific type already.
        auto it = known_map_.find(pos);
        if (it == known_map_.end()) {
            known_map_[pos] = type;
        } else if (type != CellType::WALL && it->second == CellType::WALL) {
            // Don't downgrade WALL to FREE.
        } else {
            it->second = type;
        }

        // Track bounds.
        max_x_ = std::max(max_x_, cell.x);
        min_x_ = std::min(min_x_, cell.x);
        max_y_ = std::max(max_y_, cell.y);
        min_y_ = std::min(min_y_, cell.y);

        // Track seen passable cells.
        if (type == CellType::FREE || type == CellType::START ||
            type == CellType::CONTACT_ACTIVE || type == CellType::CONTACT_PROCESSED) {
            seen_passable_cells_.insert(pos);
        }
    }
}

void MapBuilder::setStartPosition(underground_world::Position pos)
{
    start_position_ = pos;
}

void MapBuilder::markContactProcessed(int /*contact_id*/, underground_world::Position pos)
{
    auto it = known_map_.find(pos);
    if (it != known_map_.end()) {
        it->second = CellType::CONTACT_PROCESSED;
    }
}

void MapBuilder::markVisited(underground_world::Position pos)
{
    visited_.insert(pos);
}

bool MapBuilder::isKnownWall(int x, int y) const
{
    auto it = known_map_.find({x, y});
    return it != known_map_.end() && it->second == CellType::WALL;
}

bool MapBuilder::isKnownPassable(int x, int y) const
{
    auto it = known_map_.find({x, y});
    if (it == known_map_.end()) {
        return false;
    }
    return it->second != CellType::WALL && it->second != CellType::UNKNOWN;
}

bool MapBuilder::isExplored(int x, int y) const
{
    return visited_.count({x, y}) > 0;
}

bool MapBuilder::hasActiveContact(int x, int y) const
{
    auto it = known_map_.find({x, y});
    return it != known_map_.end() && it->second == CellType::CONTACT_ACTIVE;
}

bool MapBuilder::isContactProcessed(int x, int y) const
{
    auto it = known_map_.find({x, y});
    return it != known_map_.end() && it->second == CellType::CONTACT_PROCESSED;
}

std::vector<underground_world::Position> MapBuilder::getFrontierCells() const
{
    // Find unvisited passable cells adjacent to visited cells.
    std::vector<underground_world::Position> frontier;

    // Directions: UP, DOWN, LEFT, RIGHT.
    constexpr int DX[4] = {0, 0, -1, 1};
    constexpr int DY[4] = {-1, 1, 0, 0};

    for (const auto& pos : visited_) {
        for (int d = 0; d < 4; ++d) {
            underground_world::Position neighbor{pos.x + DX[d], pos.y + DY[d]};
            if (visited_.count(neighbor) > 0) {
                continue; // Already visited.
            }
            auto it = known_map_.find(neighbor);
            if (it != known_map_.end() && it->second != CellType::WALL) {
                frontier.push_back(neighbor);
            }
        }
    }

    return frontier;
}

std::vector<underground_world::Position> MapBuilder::getUnvisitedReachable() const
{
    // BFS from start to find all reachable unvisited passable cells.
    std::vector<underground_world::Position> result;
    std::set<underground_world::Position> visited;
    std::queue<underground_world::Position> q;

    q.push(start_position_);
    visited.insert(start_position_);

    constexpr int DX[4] = {0, 0, -1, 1};
    constexpr int DY[4] = {-1, 1, 0, 0};

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        for (int d = 0; d < 4; ++d) {
            underground_world::Position neighbor{current.x + DX[d], current.y + DY[d]};
            if (visited.count(neighbor) > 0) {
                continue;
            }
            auto it = known_map_.find(neighbor);
            if (it == known_map_.end()) {
                continue; // Unknown cell, skip.
            }
            if (it->second == CellType::WALL) {
                continue;
            }
            visited.insert(neighbor);
            if (!visited_.count(neighbor)) {
                result.push_back(neighbor);
            }
            q.push(neighbor);
        }
    }

    return result;
}

bool MapBuilder::isFullyExplored() const
{
    // Count all reachable passable cells via flood fill.
    int total = countPassableCells();
    if (total == 0) {
        return false;
    }
    return static_cast<int>(seen_passable_cells_.size()) >= total;
}

std::set<underground_world::Position> MapBuilder::floodFillFrom(underground_world::Position start) const
{
    std::set<underground_world::Position> result;
    std::queue<underground_world::Position> q;

    q.push(start);
    result.insert(start);

    constexpr int DX[4] = {0, 0, -1, 1};
    constexpr int DY[4] = {-1, 1, 0, 0};

    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        for (int d = 0; d < 4; ++d) {
            underground_world::Position neighbor{current.x + DX[d], current.y + DY[d]};
            if (result.count(neighbor) > 0) {
                continue;
            }
            auto it = known_map_.find(neighbor);
            if (it == known_map_.end()) {
                continue;
            }
            if (it->second == CellType::WALL) {
                continue;
            }
            result.insert(neighbor);
            q.push(neighbor);
        }
    }

    return result;
}

int MapBuilder::countPassableCells() const
{
    auto reachable = floodFillFrom(start_position_);
    int count = 0;
    for (const auto& [pos, type] : known_map_) {
        if (reachable.count(pos) > 0 && type != CellType::WALL) {
            ++count;
        }
    }
    return count;
}

int MapBuilder::getTotalPassableCount() const
{
    return countPassableCells();
}

bool MapBuilder::hasCell(int x, int y) const
{
    return known_map_.count({x, y}) > 0;
}

std::optional<CellType> MapBuilder::getCellType(int x, int y) const
{
    auto it = known_map_.find({x, y});
    if (it == known_map_.end()) {
        return std::nullopt;
    }
    return it->second;
}

} // namespace mission_explorer
