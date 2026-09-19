// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT


#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace perimeter_miner {

/// Waypoint definition for perimeter patrol
struct Waypoint {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;      // target heading when approaching (radians)
    double approach_radius = 1.0; // approach radius (meters)

    bool operator==(const Waypoint& other) const {
        return std::abs(x - other.x) < 1e-6 &&
               std::abs(y - other.y) < 1e-6;
    }
};

/// Perimeter configuration loaded from YAML
struct PerimeterConfig {
    std::string name;
    std::vector<Waypoint> waypoints;
    bool closed_loop = true;     // closed or open perimeter loop
    double tolerance = 0.5;      // waypoint reach tolerance (meters)
    double max_speed = 2.0;      // maximum linear speed (m/s)
    double min_turn_radius = 1.0; // minimum turning radius (meters)

    /// Get total number of waypoints
    size_t waypointCount() const { return waypoints.size(); }

    /// Get waypoint at index (handles closed loop wrapping)
    const Waypoint& getWaypoint(size_t index) const {
        if (closed_loop && !waypoints.empty()) {
            index = index % waypoints.size();
        }
        return waypoints.at(index);
    }
};

/// Robot state for navigation
struct RobotState {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;       // current heading (radians)
    double linear_speed = 0.0;  // current linear speed (m/s)
    double angular_speed = 0.0; // current angular speed (rad/s)

    /// Compute distance to a point
    double distanceTo(double tx, double ty) const {
        return std::hypot(tx - x, ty - y);
    }

    /// Compute bearing to a point
    double bearingTo(double tx, double ty) const {
        return std::atan2(ty - y, tx - x);
    }
};

/// Movement command output
struct MoveCommand {
    double linear_x = 0.0;   // forward speed (m/s)
    double linear_y = 0.0;   // strafe speed (m/s), 0 for skid-steer
    double angular_z = 0.0;  // rotation speed (rad/s)

    /// Create zero command
    static MoveCommand zero() { return MoveCommand{0.0, 0.0, 0.0}; }

    /// Create max speed command
    static MoveCommand fullForward(double speed) {
        return MoveCommand{speed, 0.0, 0.0};
    }
};

/// Control mode enumeration
enum class ControlMode : uint8_t {
    AUTONOMOUS = 0,
    TELEOP = 1,
    HOLD = 2
};

/// Convert control mode to string
inline const char* controlModeToString(ControlMode mode) {
    switch (mode) {
        case ControlMode::AUTONOMOUS: return "AUTONOMOUS";
        case ControlMode::TELEOP:     return "TELEOP";
        case ControlMode::HOLD:       return "HOLD";
        default:                      return "UNKNOWN";
    }
}

/// Convert control mode to uint8
inline uint8_t controlModeToUint8(ControlMode mode) {
    return static_cast<uint8_t>(mode);
}

/// Convert uint8 to control mode
inline ControlMode controlModeFromUint8(uint8_t value) {
    switch (value) {
        case 0: return ControlMode::AUTONOMOUS;
        case 1: return ControlMode::TELEOP;
        case 2: return ControlMode::HOLD;
        default: return ControlMode::AUTONOMOUS;
    }
}

} // namespace perimeter_miner
