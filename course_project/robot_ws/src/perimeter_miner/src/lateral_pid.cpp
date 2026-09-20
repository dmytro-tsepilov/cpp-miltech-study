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

#include <algorithm>
#include <cstdio>

#include "perimeter_miner/perimeter_tracker.hpp"
#include "perimeter_miner/perimeter_loader.hpp"

namespace perimeter_miner
{

double LateralPID::compute(double error, double dt)
{
  if (dt <= 0.0) {
    return 0.0;
  }

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

double PurePursuit::computeCurvature(const RobotState& robot, double tx, double ty)
{
  double dx = tx - robot.x;
  double dy = ty - robot.y;
  double distance = std::hypot(dx, dy);

  if (distance < 0.01) {
    return 0.0;
  }

  // Compute look-ahead distance based on speed
  current_lookahead_ = min_lookahead_ + lookahead_gain_ * std::abs(robot.linear_speed);
  current_lookahead_ = std::clamp(current_lookahead_, min_lookahead_, max_lookahead_);

  // If target is closer than look-ahead, use target distance
  double alpha = std::atan2(dy, dx) - robot.heading;
  alpha = std::atan2(std::sin(alpha), std::cos(alpha));  // normalize

  double curvature = 2.0 * std::sin(alpha) / current_lookahead_;
  return curvature;
}

PerimeterTracker::PerimeterTracker(const PerimeterConfig& config)
  : config_(config)
  , waypoint_tolerance_(config.tolerance)
{
  initControllers();
}

void PerimeterTracker::initControllers()
{
  // Initialize lateral PID with tuned parameters
  lateral_pid_.setParameters(2.0, 0.5, 0.3);
  lateral_pid_.setLimits(1.5, 5.0);

  // Initialize pure pursuit
  pure_pursuit_.setLookahead(1.0, 3.0, 1.5);
}

void PerimeterTracker::updateRobotState(const RobotState& state)
{
  robot_state_ = state;
}

MoveCommand PerimeterTracker::decide(double dt)
{
  MoveCommand cmd = MoveCommand::zero();

  // Check if we have coverage waypoints to use
  if (coverage_mode_ && !coverage_waypoints_.empty()) {
    // Use coverage waypoints for zigzag pattern
    if (coverage_current_idx_ >= coverage_waypoints_.size()) {
      fprintf(stderr, "[TRACKER] DECIDE: Coverage complete - returning zero command\n");
      return cmd;
    }

    // Use provided dt or default to 0.02 (50Hz)
    double real_dt = (dt > 0.0) ? dt : 0.02;

    // Get current coverage target waypoint
    const auto& target = coverage_waypoints_[coverage_current_idx_];

    // DIAGNOSTIC: Log state every 100 calls
    static int decide_count = 0;
    decide_count++;
    if (decide_count <= 20 || decide_count % 100 == 0) {
      fprintf(stderr, "[TRACKER] DECIDE #%d: wp=%zu, robot=(%.3f, %.3f)\n",
              decide_count, coverage_current_idx_, robot_state_.x, robot_state_.y);
    }

    // Check if reached current waypoint
    double dist_to_target = robot_state_.distanceTo(target.x, target.y);

    if (dist_to_target < waypoint_tolerance_) {
      // Advance to next coverage waypoint
      coverage_current_idx_++;
      fprintf(stderr, "[TRACKER] *** COVERAGE ADVANCE: wp #%zu -> %zu/%zu\n",
              coverage_current_idx_ - 1, coverage_current_idx_, coverage_waypoints_.size());

      if (coverage_current_idx_ >= coverage_waypoints_.size()) {
        fprintf(stderr, "[TRACKER] *** COVERAGE COMPLETE!\n");
        return MoveCommand::zero();
      }

      // Update target to next waypoint
      const auto& new_target = coverage_waypoints_[coverage_current_idx_];

      // Compute lateral error for new segment
      double lateral_error = computeLateralErrorCoverage();

      // Compute steering (lateral PID output) using real dt
      double steering = lateral_pid_.compute(lateral_error, real_dt);

      // Compute desired heading
      double desired_heading = new_target.heading;
      double heading_error = angleDiff(robot_state_.heading, desired_heading);

      // Build command
      cmd.linear_x = coverage_config_.coverage_speed;
      cmd.angular_z = std::clamp(heading_error * 3.0 + steering * 0.5, -1.5, 1.5);

      return cmd;
    }

    // Compute lateral error to current coverage segment
    double lateral_error = computeLateralErrorCoverage();

    // Compute steering (lateral PID output) using real dt
    double steering = lateral_pid_.compute(lateral_error, real_dt);

    // Compute desired heading
    double desired_heading = target.heading;
    double heading_error = angleDiff(robot_state_.heading, desired_heading);

    // Build command
    cmd.linear_x = coverage_config_.coverage_speed;
    cmd.angular_z = std::clamp(heading_error * 3.0 + steering * 0.5, -1.5, 1.5);

    return cmd;
  }

  // Original perimeter tracking mode
  if (config_.waypointCount() == 0) {
    fprintf(stderr, "[TRACKER] DECIDE: No waypoints - returning zero command\n");
    return cmd;
  }

  // Use provided dt or default to 0.02 (50Hz)
  double real_dt = (dt > 0.0) ? dt : 0.02;

  // Get current target waypoint
  const auto& target = config_.getWaypoint(current_waypoint_idx_);

  // DIAGNOSTIC: Log state every 100 calls
  static int decide_count = 0;
  decide_count++;
  if (decide_count <= 20 || decide_count % 100 == 0) {
    fprintf(stderr, "[TRACKER] DECIDE #%d: wp=%zu, robot=(%.3f, %.3f), init=%d\n",
            decide_count, current_waypoint_idx_, robot_state_.x, robot_state_.y,
            has_initialized_);
  }

  // Check if reached current waypoint
  double dist_to_target = robot_state_.distanceTo(target.x, target.y);

  // FIX: Only advance waypoint if we have been initialized (have valid odometry)
  // This prevents premature advance when robot starts AT waypoint 0
  if (dist_to_target < waypoint_tolerance_) {
    if (!has_initialized_) {
      // First time: mark as initialized but DON'T advance yet
      // The robot is at start position - this is expected!
      if (decide_count <= 5) {
        fprintf(stderr, "[TRACKER] INIT: dist=%.4f < tol, staying on wp #%zu\n",
                dist_to_target, current_waypoint_idx_);
      }
      has_initialized_ = true;
      // Continue with same target (don't advance)
    } else {
      // DIAGNOSTIC: Log waypoint advance only once per advance event
      static size_t last_wp = static_cast<size_t>(-1);
      if (last_wp != current_waypoint_idx_) {
        fprintf(stderr, "[TRACKER] WP ADVANCE #%d: %.4f -> wp %zu->%zu\n",
                decide_count, dist_to_target, last_wp, current_waypoint_idx_);
        last_wp = current_waypoint_idx_;
      }
      // Advance to next waypoint
      if (advanceWaypoint()) {
        fprintf(stderr, "[TRACKER] *** ADVANCED: Now targeting wp #%zu\n", current_waypoint_idx_);
        // Continue with new target
      } else {
        // End of open perimeter - stop
        return MoveCommand::zero();
      }
    }
  } else if (dist_to_target < waypoint_tolerance_ * 2 && decide_count <= 20) {
    fprintf(stderr, "[TRACKER] NEAR WAYPOINT: dist=%.4f (tolerance=%.1f), wp #%zu\n",
            dist_to_target, waypoint_tolerance_, current_waypoint_idx_);
  }

  // Compute lateral error
  double lateral_error = computeLateralError();

  // Compute steering (lateral PID output) using real dt
  double steering = lateral_pid_.compute(lateral_error, real_dt);

  // Compute desired heading
  double desired_heading = computeDesiredHeading();
  double heading_error = angleDiff(robot_state_.heading, desired_heading);

  // Compute linear speed (reduce near waypoints)
  double speed_factor = std::clamp(
    1.0 - (dist_to_target / config_.max_speed) * 2.0, 0.3, 1.0);
  double linear_speed = config_.max_speed * speed_factor;

  // Build command
  cmd.linear_x = linear_speed;
  cmd.angular_z = std::clamp(heading_error * 3.0 + steering * 0.5, -1.5, 1.5);

  return cmd;
}

bool PerimeterTracker::reachedWaypoint() const
{
  if (config_.waypointCount() == 0) {
    return false;
  }

  const auto& target = config_.getWaypoint(current_waypoint_idx_);
  double dist = robot_state_.distanceTo(target.x, target.y);
  return dist < waypoint_tolerance_;
}

bool PerimeterTracker::advanceWaypoint()
{
  if (config_.waypointCount() == 0) {
    return false;
  }

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
    current_waypoint_idx_ = 0;  // reset to start
    return false;
  }
}

double PerimeterTracker::computeLateralError() const
{
  if (config_.waypointCount() == 0) {
    return 0.0;
  }

  size_t next_idx = config_.closed_loop
    ? (current_waypoint_idx_ + 1) % config_.waypointCount()
    : std::min(current_waypoint_idx_ + 1, config_.waypointCount() - 1);

  const auto& current_wp = config_.getWaypoint(current_waypoint_idx_);
  const auto& next_wp = config_.getWaypoint(next_idx);

  // Vector from current to next waypoint
  double dx = next_wp.x - current_wp.x;
  double dy = next_wp.y - current_wp.y;
  double segment_length = std::hypot(dx, dy);

  if (segment_length < 1e-6) {
    return 0.0;
  }

  // Normalize segment vector
  dx /= segment_length;
  dy /= segment_length;

  // Vector from current waypoint to robot
  double rx = robot_state_.x - current_wp.x;
  double ry = robot_state_.y - current_wp.y;

  // Lateral error = cross product (signed distance)
  return rx * dy - ry * dx;
}

double PerimeterTracker::computeLateralErrorCoverage() const
{
  // For coverage mode, compute lateral error to the segment direction
  // Use the segment direction (from current waypoint to next waypoint) as reference

  if (coverage_current_idx_ + 1 >= coverage_waypoints_.size()) {
    // Last waypoint - no next point for segment direction
    return 0.0;
  }

  const auto& current_wp = coverage_waypoints_[coverage_current_idx_];
  const auto& next_wp = coverage_waypoints_[coverage_current_idx_ + 1];

  // Vector from current to next waypoint (segment direction)
  double dx = next_wp.x - current_wp.x;
  double dy = next_wp.y - current_wp.y;
  double segment_length = std::hypot(dx, dy);

  if (segment_length < 1e-6) {
    return 0.0;
  }

  // Normalize segment vector
  dx /= segment_length;
  dy /= segment_length;

  // Vector from current waypoint to robot
  double rx = robot_state_.x - current_wp.x;
  double ry = robot_state_.y - current_wp.y;

  // Lateral error = cross product (signed distance from robot to segment)
  return rx * dy - ry * dx;
}

double PerimeterTracker::computeDesiredHeading() const
{
  if (config_.waypointCount() == 0) {
    return robot_state_.heading;
  }

  // FIX: Compute heading FROM robot TO target waypoint (not segment direction)
  const auto& target = config_.getWaypoint(current_waypoint_idx_);
  double dx = target.x - robot_state_.x;
  double dy = target.y - robot_state_.y;

  return std::atan2(dy, dx);
}

double PerimeterTracker::computeLinearSpeed() const
{
  if (config_.waypointCount() == 0) {
    return 0.0;
  }

  const auto& target = config_.getWaypoint(current_waypoint_idx_);
  double dist = robot_state_.distanceTo(target.x, target.y);

  // Reduce speed as we approach waypoint
  double factor = std::clamp(dist / (waypoint_tolerance_ * 3.0), 0.3, 1.0);
  return config_.max_speed * factor;
}

double PerimeterTracker::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double PerimeterTracker::angleDiff(double from, double to)
{
  double diff = normalizeAngle(to - from);
  return diff;
}

TrackerStatus PerimeterTracker::getStatus() const
{
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

void PerimeterTracker::setCoverageMode(const CoverageConfig& cov_config)
{
  coverage_mode_ = true;
  coverage_config_ = cov_config;

  // Generate zigzag waypoints
  coverage_waypoints_ = PerimeterLoader::generateBoustrophedonPattern(cov_config);
  coverage_current_idx_ = 0;

  fprintf(stderr, "[TRACKER] Coverage mode enabled: %zu waypoints generated\n",
          coverage_waypoints_.size());
}

bool PerimeterTracker::isCoverageComplete() const
{
  if (!coverage_mode_) {
    return false;
  }

  // Coverage is complete when we've visited all passes
  return coverage_current_idx_ >= coverage_waypoints_.size();
}

void PerimeterTracker::reset()
{
  current_waypoint_idx_ = 0;
  has_initialized_ = false;
  lateral_pid_.reset();
  robot_state_ = RobotState{};

  // Reset coverage mode
  coverage_mode_ = false;
  coverage_current_idx_ = 0;
}

}  // namespace perimeter_miner
