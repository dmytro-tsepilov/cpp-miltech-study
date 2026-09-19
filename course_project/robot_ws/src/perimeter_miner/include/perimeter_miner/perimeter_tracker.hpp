// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT


#pragma once

#include <optional>
#include <string>

#include "perimeter_miner/perimeter_config.hpp"

namespace perimeter_miner {

/// Status struct for PerimeterTracker
struct TrackerStatus {
    uint8_t mode = 0;
    uint32_t waypoint_index = 0;
    double target_x = 0.0;
    double target_y = 0.0;
    double current_x = 0.0;
    double current_y = 0.0;
    double lateral_error = 0.0;
    float speed = 0.0f;
    bool mine_detected = false;
};

/// Lateral PID controller for perimeter tracking
class LateralPID {
public:
    LateralPID() = default;

    /// Set PID parameters
    void setParameters(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

    /// Set limits
    void setLimits(double max_output, double max_integral) {
        max_output_ = max_output;
        max_integral_ = max_integral;
    }

    /// Compute control output from error
    double compute(double error, double dt);

    /// Reset integral term (anti-windup)
    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

    /// Get current integral value
    double getIntegral() const { return integral_; }

private:
    double kp_ = 1.0;
    double ki_ = 0.1;
    double kd_ = 0.05;
    double max_output_ = 1.0;
    double max_integral_ = 10.0;

    double integral_ = 0.0;
    double prev_error_ = 0.0;
};

/// Pure pursuit navigation controller
class PurePursuit {
public:
    PurePursuit() = default;

    /// Set look-ahead distance
    void setLookahead(double min_la, double max_la, double gain) {
        min_lookahead_ = min_la;
        max_lookahead_ = max_la;
        lookahead_gain_ = gain;
    }

    /// Compute curvature for target point
    double computeCurvature(const RobotState& robot, double tx, double ty);

    /// Get current look-ahead distance
    double getLookahead() const { return current_lookahead_; }

private:
    double min_lookahead_ = 1.0;
    double max_lookahead_ = 3.0;
    double lookahead_gain_ = 1.5;
    double current_lookahead_ = 1.0;
};

/// Main perimeter tracking controller
class PerimeterTracker {
public:
    /// Create tracker with configuration
    explicit PerimeterTracker(const PerimeterConfig& config);

    /// Update robot state from odometry/sensor data
    void updateRobotState(const RobotState& state);

    /// Make navigation decision based on current state
    MoveCommand decide();

    /// Check if current waypoint is reached
    bool reachedWaypoint() const;

    /// Advance to next waypoint
    bool advanceWaypoint();

    /// Get current status
    TrackerStatus getStatus() const;

    /// Get perimeter configuration
    const PerimeterConfig& getConfig() const { return config_; }

    /// Reset tracker to initial state
    void reset();

    // Utility functions (public for testing)
    static double normalizeAngle(double angle);
    static double angleDiff(double from, double to);

private:
    PerimeterConfig config_;
    RobotState robot_state_;
    size_t current_waypoint_idx_ = 0;

    // Controllers
    LateralPID lateral_pid_;
    PurePursuit pure_pursuit_;

    // Control parameters
    double linear_speed_ = 0.5;     // default forward speed (m/s)
    double waypoint_tolerance_ = 0.5;

    // PID initialization
    void initControllers();

    // Compute lateral error from current position to next segment
    double computeLateralError() const;

    // Compute desired heading for waypoint approach
    double computeDesiredHeading() const;

    // Compute linear speed based on proximity to waypoint
    double computeLinearSpeed() const;
};

} // namespace perimeter_miner
