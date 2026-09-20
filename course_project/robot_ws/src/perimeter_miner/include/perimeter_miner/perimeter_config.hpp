// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace perimeter_miner
{

/// Waypoint definition for perimeter patrol
struct Waypoint
{
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;         // target heading when approaching (radians)
  double approach_radius = 1.0;  // approach radius (meters)

  bool operator == (const Waypoint& other) const
  {
    return std::abs(x - other.x) < 1e-6 &&
           std::abs(y - other.y) < 1e-6;
  }
};

/// Perimeter configuration loaded from YAML
struct PerimeterConfig
{
  std::string name;
  std::vector<Waypoint> waypoints;
  bool closed_loop = true;       // closed or open perimeter loop
  double tolerance = 0.5;        // waypoint reach tolerance (meters)
  double max_speed = 2.0;        // maximum linear speed (m/s)
  double min_turn_radius = 1.0;  // minimum turning radius (meters)

  // Coverage mode bounding box (used when type=coverage)
  struct BoundingBox {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 20.0;
    double max_y = 20.0;
    double pass_spacing = 2.0;
    double coverage_speed = 1.0;
    char scan_direction = 'X';
  } bounding_box;

  /// Get total number of waypoints
  size_t waypointCount() const {
    return waypoints.size();
  }

  /// Get waypoint at index (handles closed loop wrapping)
  /// Returns a default Waypoint if the vector is empty
  const Waypoint &getWaypoint(size_t index) const
  {
    static const Waypoint default_wp{};
    if (waypoints.empty()) {
      return default_wp;
    }
    if (closed_loop) {
      index = index % waypoints.size();
    }
    if (index >= waypoints.size()) {
      return default_wp;
    }
    return waypoints.at(index);
  }
};

/// Area coverage configuration for boustrophedon (zigzag) pattern
struct CoverageConfig
{
  // Bounding box of the area to cover
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 20.0;
  double max_y = 20.0;

  // Coverage parameters
  double pass_spacing = 2.0;    // distance between parallel passes (m)
  double coverage_speed = 1.0;  // forward speed during coverage (m/s)
  double turn_arcs = 1.0;       // arc length for U-turns at pass ends (m)

  // Coverage direction: 'X' = horizontal passes (scan left-right), 'Y' = vertical passes
  char scan_direction = 'X';

  // Starting corner: 'B'=bottom, 'T'=top
  char start_row = 'B';

  /// Compute area size
  double width() const {
    return max_x - min_x;
  }
  double height() const {
    return max_y - min_y;
  }

  /// Compute number of passes needed
  size_t numPasses() const
  {
    if (scan_direction == 'X') {
      // Horizontal passes: spacing affects Y axis
      return static_cast<size_t>(std::ceil(height() / pass_spacing)) + 1;
    } else {
      // Vertical passes: spacing affects X axis
      return static_cast<size_t>(std::ceil(width() / pass_spacing)) + 1;
    }
  }

  /// Compute coverage percentage given a set of waypoints
  double computeCoveragePercent(const std::vector<Waypoint>& waypoints) const;
};

/// Robot state for navigation
struct RobotState
{
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;         // current heading (radians)
  double linear_speed = 0.0;    // current linear speed (m/s)
  double angular_speed = 0.0;   // current angular speed (rad/s)

  /// Compute distance to a point
  double distanceTo(double tx, double ty) const
  {
    return std::hypot(tx - x, ty - y);
  }

  /// Compute bearing to a point
  double bearingTo(double tx, double ty) const
  {
    return std::atan2(ty - y, tx - x);
  }
};

/// Movement command output
struct MoveCommand
{
  double linear_x = 0.0;   // forward speed (m/s)
  double linear_y = 0.0;   // strafe speed (m/s), 0 for skid-steer
  double angular_z = 0.0;  // rotation speed (rad/s)

  /// Create zero command
  static MoveCommand zero() {
    return MoveCommand{ 0.0, 0.0, 0.0 };
  }

  /// Create max speed command
  static MoveCommand fullForward(double speed)
  {
    return MoveCommand{ speed, 0.0, 0.0 };
  }
};

/// Control mode enumeration
enum class ControlMode : uint8_t
{
  AUTONOMOUS = 0,
  TELEOP = 1,
  HOLD = 2,
  AREA_COVERAGE = 3
};

/// Convert control mode to string
inline const char *controlModeToString(ControlMode mode)
{
  switch (mode) {
  case ControlMode::AUTONOMOUS:       return "AUTONOMOUS";
  case ControlMode::TELEOP:           return "TELEOP";
  case ControlMode::HOLD:             return "HOLD";
  case ControlMode::AREA_COVERAGE:    return "AREA_COVERAGE";
  default:                            return "UNKNOWN";
  }
}

/// Convert control mode to uint8
inline uint8_t controlModeToUint8(ControlMode mode)
{
  return static_cast<uint8_t>(mode);
}

/// Convert uint8 to control mode
inline ControlMode controlModeFromUint8(uint8_t value)
{
  switch (value) {
  case 0: return ControlMode::AUTONOMOUS;
  case 1: return ControlMode::TELEOP;
  case 2: return ControlMode::HOLD;
  case 3: return ControlMode::AREA_COVERAGE;
  default: return ControlMode::AUTONOMOUS;
  }
}

}  // namespace perimeter_miner
