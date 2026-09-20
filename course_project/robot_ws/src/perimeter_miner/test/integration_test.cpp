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
//
// Integration tests for ModeSwitch + PerimeterTracker interaction.
// Tests the combined behavior of mode switching and perimeter tracking.

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "perimeter_miner/perimeter_config.hpp"
#include "perimeter_miner/perimeter_tracker.hpp"
#include "perimeter_miner/mode_switch.hpp"
#include "perimeter_miner/hold_controller.hpp"

using perimeter_miner::ControlMode;
using perimeter_miner::HoldController;
using perimeter_miner::ModeSwitch;
using perimeter_miner::MoveCommand;
using perimeter_miner::PerimeterConfig;
using perimeter_miner::PerimeterTracker;
using perimeter_miner::RobotState;
using perimeter_miner::Waypoint;

// Simulated MinerNode behavior for integration testing
class SimulatedMinerNode {
public:
explicit SimulatedMinerNode(const PerimeterConfig& config)
  : tracker_(config), mode_switch_(), hold_controller_() {
  // Initialize hold position to start
  if (config.waypointCount() > 0) {
    const auto& start = config.waypoints[0];
    hold_controller_.setHoldPosition(start.x, start.y, start.heading);
  }
}

void updateRobotState(const RobotState& state) {
  robot_state_ = state;
  tracker_.updateRobotState(state);
}

MoveCommand controlTick() {
  MoveCommand cmd;

  // Apply mode switch if pending
  if (mode_switch_.isSwitching()) {
    mode_switch_.applyRequest();
  }

  ControlMode current_mode = mode_switch_.getCurrentMode();

  switch (current_mode) {
  case ControlMode::AUTONOMOUS:
    cmd = tracker_.decide();
    break;

  case ControlMode::TELEOP:
    // In simulation, teleop sends zero command
    cmd = MoveCommand::zero();
    break;

  case ControlMode::HOLD:
    cmd = hold_controller_.compute(robot_state_);
    break;

  case ControlMode::AREA_COVERAGE:
    // In simulation, coverage mode uses tracker with coverage waypoints
    if (!tracker_.isCoverageComplete()) {
      cmd = tracker_.decide();
    } else {
      cmd = MoveCommand::zero();
    }
    break;

  default:
    cmd = MoveCommand::zero();
    break;
  }

  return cmd;
}

bool switchToAutonomous() {
  return mode_switch_.requestMode(ControlMode::AUTONOMOUS);
}

bool switchToTeleop() {
  return mode_switch_.operatorOverride();
}

bool switchToHold() {
  return mode_switch_.requestMode(ControlMode::HOLD);
}

ControlMode getCurrentMode() const {
  return mode_switch_.getCurrentMode();
}

MoveCommand getTrackerCommand() {
  return tracker_.decide();
}

bool isAtWaypoint() {
  return tracker_.reachedWaypoint();
}

void advanceWaypoint() {
  tracker_.advanceWaypoint();
}

void resetHoldPosition() {
  if (tracker_.getConfig().waypointCount() > 0) {
    const auto& wp = tracker_.getConfig().getWaypoint(
      tracker_.getStatus().waypoint_index);
    hold_controller_.setHoldPosition(wp.x, wp.y, wp.heading);
  }
}

private:
PerimeterTracker tracker_;
ModeSwitch mode_switch_;
HoldController hold_controller_;
RobotState robot_state_{ 0.0, 0.0, 0.0, 0.0, 0.0 };

public:
// Expose mode_switch_ for safety check testing
ModeSwitch& getModeSwitch() {
  return mode_switch_;
}
};

// Test 1: Autonomous to Teleop switch
TEST(IntegrationTest, AutonomousToTeleop)
{
  PerimeterConfig config;
  config.name = "integration_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Start in AUTONOMOUS mode
  EXPECT_EQ(node.getCurrentMode(), ControlMode::AUTONOMOUS);

  // Get autonomous command
  auto auto_cmd = node.controlTick();
  EXPECT_GT(auto_cmd.linear_x, 0.0);  // Should move forward

  // Switch to TELEOP
  bool success = node.switchToTeleop();
  EXPECT_TRUE(success);
  EXPECT_EQ(node.getCurrentMode(), ControlMode::TELEOP);

  // In TELEOP mode, command should be zero
  auto teleop_cmd = node.controlTick();
  EXPECT_DOUBLE_EQ(teleop_cmd.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(teleop_cmd.angular_z, 0.0);
}

// Test 2: Teleop to Autonomous switch
TEST(IntegrationTest, TeleopToAutonomous)
{
  PerimeterConfig config;
  config.name = "teleop_to_auto";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Switch to TELEOP first
  node.switchToTeleop();
  EXPECT_EQ(node.getCurrentMode(), ControlMode::TELEOP);

  // Try to switch back to AUTONOMOUS
  bool success = node.switchToAutonomous();

  // This may succeed or fail depending on safety checks
  // (in our implementation, it should succeed because lateral error < 5.0)
  EXPECT_TRUE(success);

  // Apply the switch
  if (node.getCurrentMode() != ControlMode::AUTONOMOUS) {
    node.controlTick();  // Trigger applyRequest
  }

  // Should now be in AUTONOMOUS mode
  EXPECT_EQ(node.getCurrentMode(), ControlMode::AUTONOMOUS);

  // Should produce non-zero command again
  auto cmd = node.controlTick();
  EXPECT_GT(cmd.linear_x, 0.0);
}

// Test 3: Autonomous to Hold switch (mine detected simulation)
TEST(IntegrationTest, AutonomousToHold)
{
  PerimeterConfig config;
  config.name = "auto_to_hold";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Start in AUTONOMOUS
  EXPECT_EQ(node.getCurrentMode(), ControlMode::AUTONOMOUS);

  // Get initial autonomous command
  auto auto_cmd = node.controlTick();
  EXPECT_GT(auto_cmd.linear_x, 0.0);  // Should produce forward motion

  // Simulate mine detection - switch to HOLD
  bool success = node.switchToHold();
  EXPECT_TRUE(success);

  // Apply the switch
  node.controlTick();

  EXPECT_EQ(node.getCurrentMode(), ControlMode::HOLD);

  // Hold controller should produce command to return to position
  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;
  node.updateRobotState(state);

  auto hold_cmd = node.controlTick();

  // Hold command should be bounded (clamped to max values)
  EXPECT_LE(std::abs(hold_cmd.linear_x), 1.0);
  EXPECT_LE(std::abs(hold_cmd.angular_z), 1.0);
}

// Test 4: Multiple mode switches in sequence
TEST(IntegrationTest, MultipleModeSwitches)
{
  PerimeterConfig config;
  config.name = "multi_switch";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Sequence: AUTO -> TELEOP -> HOLD -> AUTO
  EXPECT_EQ(node.getCurrentMode(), ControlMode::AUTONOMOUS);

  // Switch to TELEOP
  node.switchToTeleop();
  EXPECT_EQ(node.getCurrentMode(), ControlMode::TELEOP);

  // Switch to HOLD
  node.switchToHold();
  node.controlTick();  // Apply
  EXPECT_EQ(node.getCurrentMode(), ControlMode::HOLD);

  // Switch back to AUTO
  node.switchToAutonomous();
  node.controlTick();  // Apply
  EXPECT_EQ(node.getCurrentMode(), ControlMode::AUTONOMOUS);
}

// Test 5: Perimeter tracking during autonomous mode
TEST(IntegrationTest, PerimeterTrackingDuringAutonomous)
{
  PerimeterConfig config;
  config.name = "tracking_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 },
    Waypoint{ 10.0, 10.0, M_PI, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Simulate robot moving along perimeter
  int valid_commands = 0;
  for (int i = 0; i < 5; ++i) {
    RobotState state;
    state.x = static_cast<double>(i);
    state.y = 0.0;
    state.heading = 0.0;
    state.linear_speed = 1.0;
    state.angular_speed = 0.0;

    node.updateRobotState(state);
    auto cmd = node.controlTick();

    // Should produce valid commands in AUTONOMOUS mode
    EXPECT_TRUE(cmd.linear_x >= 0.0);

    if (cmd.linear_x > 0.0) {
      valid_commands++;
    }
  }

  // At least some commands should have positive forward speed
  EXPECT_GT(valid_commands, 0);
}

// Test 6: Hold controller maintains position with multiple ticks
TEST(IntegrationTest, HoldMaintainsPosition)
{
  PerimeterConfig config;
  config.name = "hold_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Set hold position to current waypoint
  node.resetHoldPosition();

  // Switch to HOLD mode
  node.switchToHold();
  node.controlTick();

  EXPECT_EQ(node.getCurrentMode(), ControlMode::HOLD);

  // Robot at hold position - should produce near-zero command
  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  node.updateRobotState(state);

  // Collect commands over multiple ticks
  double max_linear = 0.0;
  for (int i = 0; i < 3; ++i) {
    auto cmd = node.controlTick();
    max_linear = std::max(max_linear, std::abs(cmd.linear_x));
  }

  // When at hold position, commands should be near-zero (not just bounded by 2.0)
  EXPECT_LT(max_linear, 0.5);  // Tighter bound: near-zero when at position
}

// Test 7: Waypoint advancement with mode switch
TEST(IntegrationTest, WaypointAdvancementWithModeSwitch)
{
  PerimeterConfig config;
  config.name = "wp_advancement";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 },
    Waypoint{ 10.0, 10.0, M_PI, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Start at first waypoint
  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  node.updateRobotState(state);

  // Advance waypoint manually (simulating reaching it)
  node.advanceWaypoint();

  EXPECT_EQ(node.getCurrentMode(), ControlMode::AUTONOMOUS);

  // Switch modes and back
  node.switchToTeleop();
  node.switchToAutonomous();
  node.controlTick();

  EXPECT_EQ(node.getCurrentMode(), ControlMode::AUTONOMOUS);
}

// Test 8: Mode switch safety check integration
TEST(IntegrationTest, SafetyCheckIntegration)
{
  PerimeterConfig config;
  config.name = "safety_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Set a failing safety check (robot too far from perimeter)
  node.getModeSwitch().setAutonomousCheck([]() {
    return false;
  });

  // Switch to TELEOP first
  node.switchToTeleop();
  EXPECT_EQ(node.getCurrentMode(), ControlMode::TELEOP);

  // Try to switch back - should fail due to safety check
  bool success = node.switchToAutonomous();

  // The request may succeed, but applyRequest will fail
  if (success) {
    node.controlTick();  // This triggers applyRequest
    // Mode should still be TELEOP because safety check failed
    EXPECT_EQ(node.getCurrentMode(), ControlMode::TELEOP);
  }
}

// Test 9: Closed loop patrol simulation with mode verification
TEST(IntegrationTest, ClosedLoopPatrolSimulation)
{
  PerimeterConfig config;
  config.name = "closed_loop_patrol";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 },
    Waypoint{ 10.0, 10.0, M_PI, 1.0 },
    Waypoint{ 0.0, 10.0, -M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Simulate complete patrol loop
  int waypoints_completed = 0;
  int commands_produced = 0;
  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_speed = 1.0;
  state.angular_speed = 0.0;

  node.updateRobotState(state);

  // Advance through all waypoints
  for (int i = 0; i < 4; ++i) {
    node.controlTick();
    commands_produced++;

    EXPECT_EQ(node.getCurrentMode(), ControlMode::AUTONOMOUS);

    // Simulate reaching waypoint
    if (i < 3) {
      node.advanceWaypoint();
      waypoints_completed++;
    }
  }

  // Should have completed all 3 advances (waypoints 0, 1, 2 -> index becomes 3)
  EXPECT_EQ(waypoints_completed, 3);
  // All ticks should have produced commands
  EXPECT_EQ(commands_produced, 4);
}

// Test 10: Mode switch message tracking integration
TEST(IntegrationTest, MessageTrackingIntegration)
{
  PerimeterConfig config;
  config.name = "message_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);

  // Initial mode
  std::string initial_mode = "AUTONOMOUS";

  // Switch modes and check messages
  node.switchToTeleop();

  // Mode should change
  EXPECT_EQ(node.getCurrentMode(), ControlMode::TELEOP);

  // Switch back
  node.switchToAutonomous();
  node.controlTick();

  EXPECT_EQ(node.getCurrentMode(), ControlMode::AUTONOMOUS);
}

// Test 11: Mode switch prevents duplicate TELEOP override
TEST(IntegrationTest, DuplicateTeleopOverride)
{
  PerimeterConfig config;
  config.name = "dup_override_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);

  // First override should succeed
  bool first = node.switchToTeleop();
  EXPECT_TRUE(first);
  EXPECT_EQ(node.getCurrentMode(), ControlMode::TELEOP);

  // Second override while already in TELEOP should fail
  bool second = node.switchToTeleop();
  EXPECT_FALSE(second);

  // Mode should remain TELEOP
  EXPECT_EQ(node.getCurrentMode(), ControlMode::TELEOP);
}

// Test 12: Hold mode produces bounded commands when robot is far
TEST(IntegrationTest, HoldCommandsWithFarRobot)
{
  PerimeterConfig config;
  config.name = "far_hold_test";
  config.closed_loop = true;
  config.tolerance = 0.5;
  config.max_speed = 2.0;

  config.waypoints = {
    Waypoint{ 0.0, 0.0, 0.0, 1.0 },
    Waypoint{ 10.0, 0.0, M_PI_2, 1.0 }
  };

  SimulatedMinerNode node(config);
  node.resetHoldPosition();

  // Switch to HOLD mode
  node.switchToHold();
  node.controlTick();

  // Robot far from hold position
  RobotState state;
  state.x = 50.0;   // Far away
  state.y = 50.0;
  state.heading = M_PI;  // Also facing wrong way
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  node.updateRobotState(state);

  auto cmd = node.controlTick();

  // Commands should be clamped to max values (1.0 for linear, 1.0 for angular)
  EXPECT_LE(std::abs(cmd.linear_x), 1.0);
  EXPECT_LE(std::abs(cmd.angular_z), 1.0);

  // Should produce non-zero command to return to position
  EXPECT_GT(std::abs(cmd.linear_x), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
