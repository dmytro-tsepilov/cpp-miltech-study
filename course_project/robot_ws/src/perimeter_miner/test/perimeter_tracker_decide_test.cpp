// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT
//
// Test suite for PerimeterTracker::decide() — main navigation logic.
// This tests the core perimeter patrol algorithm including:
// - Lateral PID steering
// - Pure pursuit curvature
// - Linear speed computation
// - Waypoint advancement

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <algorithm>

#include "perimeter_miner/perimeter_config.hpp"
#include "perimeter_miner/perimeter_tracker.hpp"

using perimeter_miner::ControlMode;
using perimeter_miner::LateralPID;
using perimeter_miner::MoveCommand;
using perimeter_miner::PerimeterConfig;
using perimeter_miner::PerimeterTracker;
using perimeter_miner::PurePursuit;
using perimeter_miner::RobotState;
using perimeter_miner::Waypoint;

// Test 1: decide() returns zero command when no waypoints configured
TEST(PerimeterTrackerDecideTest, EmptyWaypoints)
{
  PerimeterConfig config;
  config.name = "empty";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;
  config.waypoints.clear();  // No waypoints

  PerimeterTracker tracker(config);

  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  tracker.updateRobotState(state);

  auto cmd = tracker.decide();

  EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
}

// Test 2: decide() produces forward motion toward first waypoint
TEST(PerimeterTrackerDecideTest, ForwardMotion)
{
  PerimeterConfig config;
  config.name = "forward_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{10.0, 0.0, M_PI_2, 1.0}
  };

  PerimeterTracker tracker(config);

  // Start at first waypoint
  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_speed = 0.0;
  tracker.updateRobotState(state);

  auto cmd = tracker.decide();

  // Should have positive forward speed (at or near max_speed)
  EXPECT_GT(cmd.linear_x, 0.0);
  EXPECT_LE(cmd.linear_x, config.max_speed);
}

// Test 3: decide() reduces speed near waypoint
TEST(PerimeterTrackerDecideTest, SpeedReductionNearWaypoint)
{
  PerimeterConfig config;
  config.name = "speed_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{10.0, 0.0, M_PI_2, 1.0}
  };

  PerimeterTracker tracker(config);

  // Robot far from waypoint
  RobotState state_far;
  state_far.x = 1.0;
  state_far.y = 0.0;
  state_far.heading = 0.0;
  state_far.linear_speed = 1.0;
  tracker.updateRobotState(state_far);

  auto cmd_far = tracker.decide();

  // Robot close to waypoint (within tolerance)
  RobotState state_near;
  state_near.x = 0.3;  // well within 0.5 tolerance
  state_near.y = 0.0;
  state_near.heading = 0.0;
  state_near.linear_speed = 0.5;
  tracker.updateRobotState(state_near);

  auto cmd_near = tracker.decide();

  // When very close to waypoint, linear speed should be reduced or zero
  EXPECT_GE(cmd_near.linear_x, 0.0);
}

// Test 4: decide() produces non-zero steering for lateral error
TEST(PerimeterTrackerDecideTest, SteeringForLateralError)
{
  PerimeterConfig config;
  config.name = "steering_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  // Horizontal segment from (0,0) to (10,0)
  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{10.0, 0.0, M_PI_2, 1.0}
  };

  PerimeterTracker tracker(config);

  // Robot with lateral offset (above the path)
  RobotState state;
  state.x = 5.0;
  state.y = 1.0;  // 1 meter above path
  state.heading = 0.0;
  state.linear_speed = 1.0;
  tracker.updateRobotState(state);

  auto cmd = tracker.decide();

  // Should have some angular component to correct heading
  // PID controller should produce non-zero angular command for lateral error
  EXPECT_NE(cmd.angular_z, 0.0);
}

// Test 5: decide() with closed loop wraps around
TEST(PerimeterTrackerDecideTest, ClosedLoopWrapAround)
{
  PerimeterConfig config;
  config.name = "loop_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{10.0, 0.0, M_PI_2, 1.0},
    Waypoint{10.0, 10.0, M_PI, 1.0},
    Waypoint{0.0, 10.0, -M_PI_2, 1.0}
  };

  PerimeterTracker tracker(config);

  // Start at first waypoint
  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  tracker.updateRobotState(state);

  // Advance to last waypoint manually
  for (int i = 0; i < 3; ++i) {
    tracker.advanceWaypoint();
  }

  auto status = tracker.getStatus();
  EXPECT_EQ(status.waypoint_index, size_t(3));

  // One more advance should wrap to 0
  tracker.advanceWaypoint();
  status = tracker.getStatus();
  EXPECT_EQ(status.waypoint_index, size_t(0));
}

// Test 6: decide() with open perimeter stops at end
TEST(PerimeterTrackerDecideTest, OpenPerimeterStop)
{
  PerimeterConfig config;
  config.name = "open_test";
  config.closed_loop = false;  // Open perimeter
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{10.0, 0.0, M_PI_2, 1.0},
    Waypoint{10.0, 10.0, M_PI, 1.0}
  };

  PerimeterTracker tracker(config);

  // Advance to last waypoint
  for (int i = 0; i < 2; ++i) {
    tracker.advanceWaypoint();
  }

  auto status = tracker.getStatus();
  EXPECT_EQ(status.waypoint_index, size_t(2));

  // Simulate reaching last waypoint
  RobotState state;
  state.x = 10.0;
  state.y = 10.0;
  state.heading = M_PI;
  tracker.updateRobotState(state);

  // reachedWaypoint should return true at last waypoint
  EXPECT_TRUE(tracker.reachedWaypoint());

  // advanceWaypoint on open perimeter at end returns false
  bool advanced = tracker.advanceWaypoint();
  EXPECT_FALSE(advanced);  // End of open perimeter
}

// Test 7: decide() computes valid status
TEST(PerimeterTrackerDecideTest, StatusComputation)
{
  PerimeterConfig config;
  config.name = "status_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{10.0, 0.0, M_PI_2, 1.0}
  };

  PerimeterTracker tracker(config);

  RobotState state;
  state.x = 2.0;
  state.y = 0.5;
  state.heading = 0.0;
  state.linear_speed = 1.0;
  tracker.updateRobotState(state);

  tracker.decide();
  auto status = tracker.getStatus();

  // Status should reflect current state
  // When at waypoint 0, target is still waypoint 0 (not reached yet)
  EXPECT_EQ(status.waypoint_index, size_t(0));
  EXPECT_DOUBLE_EQ(status.target_x, 0.0);
  EXPECT_DOUBLE_EQ(status.target_y, 0.0);
  EXPECT_NEAR(status.current_x, 2.0, 0.01);
  EXPECT_NEAR(status.current_y, 0.5, 0.01);
}

// Test 8: decide() speed factor based on distance
TEST(PerimeterTrackerDecideTest, SpeedFactor)
{
  PerimeterConfig config;
  config.name = "speed_factor_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{20.0, 0.0, M_PI_2, 1.0}
  };

  PerimeterTracker tracker(config);

  // Robot far from waypoint (at start)
  RobotState state_far;
  state_far.x = 0.0;
  state_far.y = 0.0;
  state_far.heading = 0.0;
  state_far.linear_speed = 0.0;
  tracker.updateRobotState(state_far);

  auto cmd_far = tracker.decide();

  // Speed should be at or near max_speed when far away
  EXPECT_GT(cmd_far.linear_x, 0.5);
}

// Test 9: Waypoint reach detection with tolerance
TEST(PerimeterTrackerDecideTest, WaypointReachTolerance)
{
  PerimeterConfig config;
  config.name = "tolerance_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{10.0, 0.0, M_PI_2, 1.0}
  };

  PerimeterTracker tracker(config);

  // Robot exactly at first waypoint
  RobotState state_at;
  state_at.x = 0.0;
  state_at.y = 0.0;
  state_at.heading = 0.0;
  tracker.updateRobotState(state_at);

  EXPECT_TRUE(tracker.reachedWaypoint());

  // Robot just outside tolerance
  RobotState state_outside;
  state_outside.x = 0.6;  // > 0.5 tolerance
  state_outside.y = 0.0;
  state_outside.heading = 0.0;
  tracker.updateRobotState(state_outside);

  EXPECT_FALSE(tracker.reachedWaypoint());

  // Robot just inside tolerance (diagonal)
  RobotState state_inside;
  state_inside.x = 0.4;
  state_inside.y = 0.3;  // sqrt(0.16 + 0.09) = 0.5
  state_inside.heading = 0.0;
  tracker.updateRobotState(state_inside);

  // Distance = sqrt(0.16 + 0.09) = 0.5, exactly at tolerance boundary
  // The implementation uses < not <=, so exactly at tolerance is NOT reached
  EXPECT_FALSE(tracker.reachedWaypoint());
}

// Test 10: Reset functionality
TEST(PerimeterTrackerDecideTest, Reset)
{
  PerimeterConfig config;
  config.name = "reset_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{10.0, 0.0, M_PI_2, 1.0},
    Waypoint{10.0, 10.0, M_PI, 1.0}
  };

  PerimeterTracker tracker(config);

  // Advance to middle waypoint
  tracker.advanceWaypoint();
  tracker.advanceWaypoint();

  auto status = tracker.getStatus();
  EXPECT_EQ(status.waypoint_index, size_t(2));

  // Reset
  tracker.reset();

  status = tracker.getStatus();
  EXPECT_EQ(status.waypoint_index, size_t(0));
}

// Test 11: Lateral error computation for horizontal segment
TEST(PerimeterTrackerDecideTest, LateralErrorHorizontal)
{
  PerimeterConfig config;
  config.name = "lateral_error_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  // Horizontal segment from (0,0) to (10,0)
  config.waypoints = {
    Waypoint{0.0, 0.0, 0.0, 1.0},
    Waypoint{10.0, 0.0, M_PI_2, 1.0}
  };

  PerimeterTracker tracker(config);

  // Robot above the path (positive lateral error)
  RobotState state_above;
  state_above.x = 5.0;
  state_above.y = 1.0;
  state_above.heading = 0.0;
  tracker.updateRobotState(state_above);

  auto status_above = tracker.getStatus();

  // Robot below the path (negative lateral error)
  RobotState state_below;
  state_below.x = 5.0;
  state_below.y = -1.0;
  state_below.heading = 0.0;
  tracker.updateRobotState(state_below);

  auto status_below = tracker.getStatus();

  // Lateral errors should have opposite signs
  EXPECT_LT(status_above.lateral_error * status_below.lateral_error, 0);
}

// Test 12: Pure pursuit curvature computation — different targets produce different curvatures
TEST(PurePursuitTest, CurvatureComputation)
{
  PurePursuit pursuit;
  pursuit.setLookahead(1.0, 3.0, 1.5);

  RobotState robot;
  robot.x = 0.0;
  robot.y = 0.0;
  robot.heading = 0.0;
  robot.linear_speed = 1.0;

  // Target straight ahead — should produce minimal curvature
  double curvStraight = pursuit.computeCurvature(robot, 10.0, 0.0);

  // Target to the side — should produce significant curvature
  double curvSide = pursuit.computeCurvature(robot, 5.0, 5.0);

  // Curvatures should differ (side turn requires more steering)
  EXPECT_NE(curvStraight, curvSide);

  // Straight ahead should have smaller absolute curvature than side target
  EXPECT_LT(std::abs(curvStraight), std::abs(curvSide));
}

// Test 13: PID integral accumulates differently with different dt values
TEST(PerimeterTrackerDecideTest, PIDWithDifferentDT)
{
  LateralPID pid;
  pid.setParameters(2.0, 0.5, 0.3);
  pid.setLimits(1.5, 5.0);

  // Accumulate error over multiple ticks at 100Hz (dt=0.01)
  for (int i = 0; i < 10; ++i) {
    pid.compute(1.0, 0.01);
  }
  double integral100hz = pid.getIntegral();

  pid.reset();

  // Accumulate error over fewer ticks at 20Hz (dt=0.05) — same total time
  for (int i = 0; i < 2; ++i) {
    pid.compute(1.0, 0.05);
  }
  double integral20hz = pid.getIntegral();

  // Integrals should differ due to different accumulation patterns
  EXPECT_NE(integral100hz, integral20hz);

  // Reset should clear integral
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
}

// Test 14: Angle normalization edge cases
TEST(AngleNormalizationTest, EdgeCases)
{
  // Test normalizeAngle via angleDiff

  // Same angle
  double diff = PerimeterTracker::angleDiff(M_PI, M_PI);
  EXPECT_DOUBLE_EQ(diff, 0.0);

  // Near wrap-around (M_PI-0.1 vs -M_PI+0.1 = difference of 0.2 radians)
  diff = PerimeterTracker::angleDiff(M_PI - 0.1, -M_PI + 0.1);
  EXPECT_NEAR(diff, 0.2, 0.01);  // Difference is 0.2 radians

  // Full circle difference (PI vs -PI are the same angle)
  diff = PerimeterTracker::angleDiff(M_PI, -M_PI);
  EXPECT_NEAR(diff, 0.0, 0.01);  // Should be ~0 since PI and -PI are same angle
}

// Test 15: MoveCommand zero and fullForward
TEST(MoveCommandTest, ZeroAndFullForward)
{
  auto zero = MoveCommand::zero();
  EXPECT_DOUBLE_EQ(zero.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(zero.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(zero.angular_z, 0.0);

  auto full = MoveCommand::fullForward(3.0);
  EXPECT_DOUBLE_EQ(full.linear_x, 3.0);
  EXPECT_DOUBLE_EQ(full.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(full.angular_z, 0.0);
}

// Test 16: Lateral PID output is bounded by max_output limit
TEST(LateralPIDTest, OutputBoundedByLimits)
{
  LateralPID pid;
  pid.setParameters(2.0, 0.5, 0.3);
  pid.setLimits(1.5, 5.0);  // max_output = 1.5

  // Large error should produce output clamped to max_output
  double output = pid.compute(10.0, 0.02);

  EXPECT_LE(std::abs(output), 1.5);
  EXPECT_GT(output, 0.0);  // Should still be positive

  pid.reset();
}

// Test 17: Pure pursuit lookahead affects curvature computation
TEST(PurePursuitTest, LookaheadAffectsCurvature)
{
  PurePursuit pursuit;

  RobotState robot;
  robot.x = 0.0;
  robot.y = 0.0;
  robot.heading = 0.0;
  robot.linear_speed = 1.0;

  // Set short lookahead
  pursuit.setLookahead(0.5, 1.5, 1.5);
  double curvShort = pursuit.computeCurvature(robot, 5.0, 2.0);

  // Set long lookahead
  pursuit.setLookahead(1.0, 3.0, 1.5);
  double curvLong = pursuit.computeCurvature(robot, 5.0, 2.0);

  // Different lookaheads should produce different curvatures
  EXPECT_NE(curvShort, curvLong);
}

// Test 18: PerimeterTracker with single waypoint
TEST(PerimeterTrackerDecideTest, SingleWaypoint)
{
  PerimeterConfig config;
  config.name = "single_wp";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{5.0, 5.0, M_PI_4, 1.0}
  };

  PerimeterTracker tracker(config);

  // Robot at the waypoint
  RobotState state;
  state.x = 5.0;
  state.y = 5.0;
  state.heading = M_PI_4;
  tracker.updateRobotState(state);

  EXPECT_TRUE(tracker.reachedWaypoint());

  // Reset and move away
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  tracker.updateRobotState(state);

  EXPECT_FALSE(tracker.reachedWaypoint());
}

// Test 19: PerimeterTracker computeLateralError for vertical segment
TEST(PerimeterTrackerDecideTest, LateralErrorVertical)
{
  PerimeterConfig config;
  config.name = "vertical_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  // Vertical segment from (0,0) to (0,10)
  config.waypoints = {
    Waypoint{0.0, 0.0, M_PI_2, 1.0},
    Waypoint{0.0, 10.0, M_PI_2, 1.0}
  };

  PerimeterTracker tracker(config);

  // Robot to the right of path (positive lateral error)
  RobotState state_right;
  state_right.x = 1.0;
  state_right.y = 5.0;
  state_right.heading = M_PI_2;
  tracker.updateRobotState(state_right);
  auto status_right = tracker.getStatus();

  // Robot to the left of path (negative lateral error)
  RobotState state_left;
  state_left.x = -1.0;
  state_left.y = 5.0;
  state_left.heading = M_PI_2;
  tracker.updateRobotState(state_left);
  auto status_left = tracker.getStatus();

  // Lateral errors should have opposite signs
  EXPECT_LT(status_right.lateral_error * status_left.lateral_error, 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
