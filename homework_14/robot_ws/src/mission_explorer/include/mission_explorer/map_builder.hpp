#pragma once

#include <climits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "underground_world/msg/local_scan.hpp"
#include "underground_world/scenario.hpp"

namespace mission_explorer {

/// Internal cell type used by the MapBuilder.
enum class CellType {
    UNKNOWN,
    WALL,
    FREE,
    START,
    CONTACT_ACTIVE,
    CONTACT_PROCESSED
};

/// String representation for debugging.
constexpr const char* cell_type_to_string(CellType t) {
    switch (t) {
        case CellType::UNKNOWN: return "?";
        case CellType::WALL: return "#";
        case CellType::FREE: return ".";
        case CellType::START: return "S";
        case CellType::CONTACT_ACTIVE: return "C";
        case CellType::CONTACT_PROCESSED: return "x";
    }
    return "?";
}

class MapBuilder {
public:
    MapBuilder() = default;

    /// Update internal map from a LocalScan observation.
    void updateFromScan(const underground_world::msg::LocalScan& scan);

    /// Set the start position (from scenario or first scan).
    void setStartPosition(underground_world::Position pos);

    /// Mark that a contact has been processed at the given position.
    void markContactProcessed(int contact_id, underground_world::Position pos);

    /// Mark a cell as visited (robot has been there).
    void markVisited(underground_world::Position pos);

    // --- Query methods ---

    bool isKnownWall(int x, int y) const;
    bool isKnownPassable(int x, int y) const;
    bool isExplored(int x, int y) const;  // robot has visited
    bool hasActiveContact(int x, int y) const;
    bool isContactProcessed(int x, int y) const;

    /// Get all known passable cells adjacent to explored boundary (frontier).
    std::vector<underground_world::Position> getFrontierCells() const;

    /// Get all unvisited reachable passable cells.
    std::vector<underground_world::Position> getUnvisitedReachable() const;

    /// Check if all reachable passable cells have been seen in a LocalScan.
    bool isFullyExplored() const;

    /// Get start position.
    underground_world::Position getStartPosition() const { return start_position_; }

    /// Get count of unique passable cells that have been seen.
    int getSeenPassableCount() const { return static_cast<int>(seen_passable_cells_.size()); }

    /// Get total passable cell count (including unseen).
    int getTotalPassableCount() const;

    /// Check if a position is within known map bounds.
    bool hasCell(int x, int y) const;

    /// Get the known cell type at a position.
    std::optional<CellType> getCellType(int x, int y) const;

private:
    underground_world::Position start_position_{0, 0};
    std::map<underground_world::Position, CellType> known_map_;
    std::set<underground_world::Position> visited_;
    std::set<underground_world::Position> seen_passable_cells_;

    // Bounds tracking for efficient queries.
    int max_x_ = 0;
    int min_x_ = INT32_MAX;
    int max_y_ = 0;
    int min_y_ = INT32_MAX;

    /// Flood-fill to count reachable passable cells from start.
    std::set<underground_world::Position> floodFillFrom(underground_world::Position start) const;

    /// Count total passable cells via flood fill from start.
    int countPassableCells() const;
};

} // namespace mission_explorer
