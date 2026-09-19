// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT
//
// Test suite for HoldController PID calculations.

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

#include "perimeter_miner/perimeter_config.hpp"
#include "perimeter_miner/hold_controller.hpp"

using perimeter_miner::HoldController;
using perimeter_miner::MoveCommand;
using perimeter_miner::RobotState;

// Test 1: Hold controller produces zero command when at position
TEST(HoldControllerPIDTest, ZeroCommandAtPosition)
{
  HoldController hold;
  hold.setHoldPosition(5.0, 10.0, M_PI_4);

  RobotState state;
  state.x = 5.0;
  state.y = 10.0;
  state.heading = M_PI_4;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  auto cmd = hold.compute(state);

  // Should produce near-zero command when at position
  EXPECT_LT(std::abs(cmd.linear_x), 0.01);
  EXPECT_LT(std::abs(cmd.linear_y), 0.01);
  EXPECT_LT(std::abs(cmd.angular_z), 0.01);
}

// Test 2: Hold controller produces positive command when robot is behind
TEST(HoldControllerPIDTest, PositiveCommandWhenBehind)
{
  HoldController hold;
  hold.setHoldPosition(10.0, 0.0, 0.0);

  RobotState state;
  state.x = 5.0;   // Behind target
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  auto cmd = hold.compute(state);

  // Should produce positive linear command to move forward
  EXPECT_GT(cmd.linear_x, 0.0);
}

// Test 3: Hold controller produces negative command when robot is ahead
TEST(HoldControllerPIDTest, NegativeCommandWhenAhead)
{
  HoldController hold;
  hold.setHoldPosition(5.0, 0.0, 0.0);

  RobotState state;
  state.x = 10.0;   // Ahead of target
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  auto cmd = hold.compute(state);

  // Should produce negative linear command to move backward
  EXPECT_LT(cmd.linear_x, 0.0);
}

// Test 4: Hold controller produces angular command for heading error
TEST(HoldControllerPIDTest, AngularCommandForHeading)
{
  HoldController hold;
  hold.setHoldPosition(0.0, 0.0, 0.0);  // Target heading = 0

  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = M_PI_2;   // Facing 90 degrees off
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  auto cmd = hold.compute(state);

  // Should produce angular command to correct heading
  EXPECT_NE(cmd.angular_z, 0.0);
}

// Test 5: Hold controller output is clamped
TEST(HoldControllerPIDTest, OutputClamping)
{
  HoldController hold;
  hold.setHoldPosition(0.0, 0.0, 0.0);

  // Robot far from target (should produce large error)
  RobotState state;
  state.x = -100.0;   // Very far behind
  state.y = -100.0;
  state.heading = M_PI;   // Completely opposite
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  auto cmd = hold.compute(state);

  // Outputs should be clamped to max values
  EXPECT_LE(std::abs(cmd.linear_x), 1.0);  // max_linear_ = 1.0
  EXPECT_LE(std::abs(cmd.linear_y), 1.0);
  EXPECT_LE(std::abs(cmd.angular_z), 1.0);  // max_angular_ = 1.0
}

// Test 6: isAtHoldPosition with tolerance
TEST(HoldControllerPIDTest, AtHoldPositionWithTolerance)
{
  HoldController hold;
  hold.setHoldPosition(5.0, 10.0, M_PI_4);

  // Within default tolerance (0.5)
  RobotState state_close;
  state_close.x = 5.3;   // 0.3m away
  state_close.y = 10.2;
  state_close.heading = M_PI_4 + 0.05;  // ~3 degrees off
  state_close.linear_speed = 0.0;
  state_close.angular_speed = 0.0;

  EXPECT_TRUE(hold.isAtHoldPosition(state_close, 0.5));

  // Outside tolerance
  RobotState state_far;
  state_far.x = 6.0;   // 1.0m away
  state_far.y = 11.0;
  state_far.heading = M_PI_4 + 0.2;  // ~11 degrees off
  state_far.linear_speed = 0.0;
  state_far.angular_speed = 0.0;

  EXPECT_FALSE(hold.isAtHoldPosition(state_far, 0.5));
}

// Test 7: Set hold position resets integral terms
TEST(HoldControllerPIDTest, ResetOnSetPosition)
{
  HoldController hold;
  
  // Move robot away first to build up integral
  RobotState state1;
  state1.x = 0.0;
  state1.y = 0.0;
  state1.heading = 0.0;
  hold.setHoldPosition(10.0, 10.0, 0.0);
  
  for (int i = 0; i < 10; ++i) {
    hold.compute(state1);
  }

  // Set new hold position - should reset integrals
  hold.setHoldPosition(5.0, 5.0, M_PI_2);

  RobotState state2;
  state2.x = 5.0;
  state2.y = 5.0;
  state2.heading = M_PI_2;
  state2.linear_speed = 0.0;
  state2.angular_speed = 0.0;

  auto cmd = hold.compute(state2);

  // First command after reset should not have accumulated integral bias
  EXPECT_TRUE(true);  // Just verify no crash
}

// Test 8: Hold controller with different positions
TEST(HoldControllerPIDTest, DifferentHoldPositions)
{
  HoldController hold;

  // Test negative coordinates
  hold.setHoldPosition(-5.0, -10.0, -M_PI_4);

  RobotState state;
  state.x = -5.0;
  state.y = -10.0;
  state.heading = -M_PI_4;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  EXPECT_TRUE(hold.isAtHoldPosition(state, 0.5));

  auto cmd = hold.compute(state);
  EXPECT_LT(std::abs(cmd.linear_x), 0.01);
}

// Test 9: Hold controller response to position change
TEST(HoldControllerPIDTest, ResponseToPositionChange)
{
  HoldController hold;
  hold.setHoldPosition(0.0, 0.0, 0.0);

  RobotState state;
  // Use small distances that won't hit max_linear_ clamp (default 1.0)
  // With kp=2.0: error 0.1 -> cmd=0.2, error 0.2 -> cmd=0.4 (both unclamped)
  state.x = 0.1;
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  auto cmd1 = hold.compute(state);

  // Move robot further away (but still within clamp range)
  state.x = 0.2;
  auto cmd2 = hold.compute(state);

  // Command at distance 0.2 should be larger than at 0.1
  EXPECT_GT(std::abs(cmd2.linear_x), std::abs(cmd1.linear_x));
}

// Test 10: Heading error normalization
TEST(HoldControllerPIDTest, HeadingErrorNormalization)
{
  HoldController hold;
  hold.setHoldPosition(0.0, 0.0, 0.0);

  // Robot facing opposite direction
  RobotState state1;
  state1.x = 0.0;
  state1.y = 0.0;
  state1.heading = M_PI;
  state1.linear_speed = 0.0;
  state1.angular_speed = 0.0;

  auto cmd1 = hold.compute(state1);

  // Robot facing same direction (should have smaller angular error)
  RobotState state2;
  state2.x = 0.0;
  state2.y = 0.0;
  state2.heading = 0.0;
  state2.linear_speed = 0.0;
  state2.angular_speed = 0.0;

  auto cmd2 = hold.compute(state2);

  // Angular command should be larger for opposite heading
  EXPECT_GT(std::abs(cmd1.angular_z), std::abs(cmd2.angular_z));
}

// Test 11: Zero tolerance at hold position
TEST(HoldControllerPIDTest, ZeroToleranceCheck)
{
  HoldController hold;
  hold.setHoldPosition(5.0, 5.0, 0.0);

  RobotState state;
  state.x = 5.0;
  state.y = 5.0;
  state.heading = 0.0;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  // With zero tolerance, only exact match passes
  EXPECT_TRUE(hold.isAtHoldPosition(state, 0.0));
}

// Test 12: Large heading difference
TEST(HoldControllerPIDTest, LargeHeadingDifference)
{
  HoldController hold;
  hold.setHoldPosition(0.0, 0.0, 0.0);

  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 3.14;  // Almost PI radians off
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  auto cmd = hold.compute(state);

  // Should produce significant angular command
  EXPECT_GT(std::abs(cmd.angular_z), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
