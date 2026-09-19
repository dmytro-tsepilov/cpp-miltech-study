// Copyright 2026 Perimeter Miner Project
// SPDX-License-Identifier: MIT

#include "perimeter_miner/perimeter_tracker.hpp"
#include <algorithm>

namespace perimeter_miner {

double LateralPID::compute(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    // Proportional term
    double p_term = kp_ * error;

    // Integral term with anti-windup
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -max_integral_, max_integral_);
    double i_term = ki_ * integral_;

    // Derivative term
    double derivative = (error - prev_error_) / dt;
    double d_term = kd_ * derivative;

    // Sum and clamp output
    double output = p_term + i_term + d_term;
    output = std::clamp(output, -max_output_, max_output_);

    // Update previous error
    prev_error_ = error;

    return output;
}

double PurePursuit::computeCurvature(const RobotState& robot, double tx, double ty) {
    double dx = tx - robot.x;
    double dy = ty - robot.y;
    double distance = std::hypot(dx, dy);

    if (distance < 0.01) return 0.0;

    // Compute look-ahead distance based on speed
    current_lookahead_ = min_lookahead_ + lookahead_gain_ * std::abs(robot.linear_speed);
    current_lookahead_ = std::clamp(current_lookahead_, min_lookahead_, max_lookahead_);

    // If target is closer than look-ahead, use target distance
    double alpha = std::atan2(dy, dx) - robot.heading;
    alpha = std::atan2(std::sin(alpha), std::cos(alpha)); // normalize

    double curvature = 2.0 * std::sin(alpha) / current_lookahead_;
    return curvature;
}

PerimeterTracker::PerimeterTracker(const PerimeterConfig& config)
    : config_(config)
    , waypoint_tolerance_(config.tolerance) {
    initControllers();
}

void PerimeterTracker::initControllers() {
    // Initialize lateral PID with tuned parameters
    lateral_pid_.setParameters(2.0, 0.5, 0.3);
    lateral_pid_.setLimits(1.5, 5.0);

    // Initialize pure pursuit
    pure_pursuit_.setLookahead(1.0, 3.0, 1.5);
}

void PerimeterTracker::updateRobotState(const RobotState& state) {
    robot_state_ = state;
}

MoveCommand PerimeterTracker::decide() {
    MoveCommand cmd = MoveCommand::zero();

    if (config_.waypointCount() == 0) {
        return cmd;
    }

    // Get current target waypoint
    const auto& target = config_.getWaypoint(current_waypoint_idx_);

    // Check if reached current waypoint
    double dist_to_target = robot_state_.distanceTo(target.x, target.y);

    if (dist_to_target < waypoint_tolerance_) {
        // Advance to next waypoint
        if (advanceWaypoint()) {
            // Continue with new target
        } else {
            // End of open perimeter - stop
            return MoveCommand::zero();
        }
    }

    // Compute lateral error
    double lateral_error = computeLateralError();

    // Compute steering (lateral PID output)
    double steering = lateral_pid_.compute(lateral_error, 0.02); // assume 50Hz

    // Compute desired heading
    double desired_heading = computeDesiredHeading();
    double heading_error = angleDiff(robot_state_.heading, desired_heading);

    // Compute linear speed (reduce near waypoints)
    double speed_factor = std::clamp(1.0 - (dist_to_target / config_.max_speed) * 2.0, 0.3, 1.0);
    double linear_speed = config_.max_speed * speed_factor;

    // Build command
    cmd.linear_x = linear_speed;
    cmd.angular_z = std::clamp(heading_error * 3.0 + steering * 0.5, -1.5, 1.5);

    return cmd;
}

bool PerimeterTracker::reachedWaypoint() const {
    if (config_.waypointCount() == 0) return false;

    const auto& target = config_.getWaypoint(current_waypoint_idx_);
    double dist = robot_state_.distanceTo(target.x, target.y);
    return dist < waypoint_tolerance_;
}

bool PerimeterTracker::advanceWaypoint() {
    if (config_.waypointCount() == 0) return false;

    // Reset lateral PID for new segment
    lateral_pid_.reset();

    if (config_.closed_loop) {
        current_waypoint_idx_ = (current_waypoint_idx_ + 1) % config_.waypointCount();
        return true;
    } else {
        if (current_waypoint_idx_ + 1 < config_.waypointCount()) {
            ++current_waypoint_idx_;
            return true;
        }
        // End of open perimeter
        current_waypoint_idx_ = 0; // reset to start
        return false;
    }
}

double PerimeterTracker::computeLateralError() const {
    if (config_.waypointCount() == 0) return 0.0;

    size_t next_idx = config_.closed_loop
        ? (current_waypoint_idx_ + 1) % config_.waypointCount()
        : std::min(current_waypoint_idx_ + 1, config_.waypointCount() - 1);

    const auto& current_wp = config_.getWaypoint(current_waypoint_idx_);
    const auto& next_wp = config_.getWaypoint(next_idx);

    // Vector from current to next waypoint
    double dx = next_wp.x - current_wp.x;
    double dy = next_wp.y - current_wp.y;
    double segment_length = std::hypot(dx, dy);

    if (segment_length < 1e-6) return 0.0;

    // Normalize segment vector
    dx /= segment_length;
    dy /= segment_length;

    // Vector from current waypoint to robot
    double rx = robot_state_.x - current_wp.x;
    double ry = robot_state_.y - current_wp.y;

    // Lateral error = cross product (signed distance)
    return rx * dy - ry * dx;
}

double PerimeterTracker::computeDesiredHeading() const {
    if (config_.waypointCount() == 0) return robot_state_.heading;

    size_t next_idx = config_.closed_loop
        ? (current_waypoint_idx_ + 1) % config_.waypointCount()
        : std::min(current_waypoint_idx_ + 1, config_.waypointCount() - 1);

    const auto& current_wp = config_.getWaypoint(current_waypoint_idx_);
    const auto& next_wp = config_.getWaypoint(next_idx);

    return std::atan2(next_wp.y - current_wp.y, next_wp.x - current_wp.x);
}

double PerimeterTracker::computeLinearSpeed() const {
    if (config_.waypointCount() == 0) return 0.0;

    const auto& target = config_.getWaypoint(current_waypoint_idx_);
    double dist = robot_state_.distanceTo(target.x, target.y);

    // Reduce speed as we approach waypoint
    double factor = std::clamp(dist / (waypoint_tolerance_ * 3.0), 0.3, 1.0);
    return config_.max_speed * factor;
}

double PerimeterTracker::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double PerimeterTracker::angleDiff(double from, double to) {
    double diff = normalizeAngle(to - from);
    return diff;
}

TrackerStatus PerimeterTracker::getStatus() const {

    TrackerStatus status{};
    status.waypoint_index = static_cast<uint32_t>(current_waypoint_idx_);
    status.current_x = robot_state_.x;
    status.current_y = robot_state_.y;
    status.speed = static_cast<float>(robot_state_.linear_speed);

    if (config_.waypointCount() > 0) {
        const auto& target = config_.getWaypoint(current_waypoint_idx_);
        status.target_x = target.x;
        status.target_y = target.y;
        status.lateral_error = computeLateralError();
    }

    return status;
}

void PerimeterTracker::reset() {
    current_waypoint_idx_ = 0;
    lateral_pid_.reset();
    robot_state_ = RobotState{};
}

} // namespace perimeter_miner
