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

  // After reset, command should be near-zero since robot is at new hold position
  EXPECT_LT(std::abs(cmd.linear_x), 0.01);
  EXPECT_LT(std::abs(cmd.linear_y), 0.01);
  EXPECT_LT(std::abs(cmd.angular_z), 0.01);
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

// Test 12: Large heading difference produces significant angular command
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

  // Should produce significant angular command (heading_kp=3.0, error≈PI)
  // With kp=3.0 and error≈3.14: angular_z ≈ 3.0 * 3.14 = 9.42, clamped to 1.0
  EXPECT_GT(std::abs(cmd.angular_z), 0.5);  // Should be close to max (1.0)
}

// Test 13: Hold controller with zero distance error
TEST(HoldControllerPIDTest, ZeroDistanceError)
{
  HoldController hold;
  hold.setHoldPosition(0.0, 0.0, 0.0);

  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_speed = 0.0;
  state.angular_speed = 0.0;

  auto cmd = hold.compute(state);

  // All commands should be zero when at hold position
  EXPECT_NEAR(cmd.linear_x, 0.0, 0.001);
  EXPECT_NEAR(cmd.linear_y, 0.0, 0.001);
  EXPECT_NEAR(cmd.angular_z, 0.0, 0.001);
}

// Test 14: Hold controller heading correction direction
TEST(HoldControllerPIDTest, HeadingCorrectionDirection)
{
  HoldController hold;
  hold.setHoldPosition(0.0, 0.0, 0.0);  // Target heading = 0

  // Robot facing +90 degrees (should turn negative to correct)
  RobotState state_ccw;
  state_ccw.x = 0.0;
  state_ccw.y = 0.0;
  state_ccw.heading = M_PI_2;
  state_ccw.linear_speed = 0.0;
  state_ccw.angular_speed = 0.0;

  auto cmd_ccw = hold.compute(state_ccw);

  // Robot facing -90 degrees (should turn positive to correct)
  RobotState state_cw;
  state_cw.x = 0.0;
  state_cw.y = 0.0;
  state_cw.heading = -M_PI_2;
  state_cw.linear_speed = 0.0;
  state_cw.angular_speed = 0.0;

  auto cmd_cw = hold.compute(state_cw);

  // Angular commands should have opposite signs for opposite heading errors
  EXPECT_NE(cmd_ccw.angular_z, 0.0);
  EXPECT_NE(cmd_cw.angular_z, 0.0);
  EXPECT_LT(cmd_ccw.angular_z * cmd_cw.angular_z, 0);  // Opposite signs
}

// Test 15: Hold controller position tolerance uses Euclidean distance
TEST(HoldControllerPIDTest, PositionToleranceEuclidean)
{
  HoldController hold;
  hold.setHoldPosition(5.0, 10.0, 0.0);

  // Within Euclidean tolerance (distance < 0.5)
  // sqrt(0.3^2 + 0.3^2) = sqrt(0.18) ≈ 0.424 < 0.5
  RobotState state_within;
  state_within.x = 5.3;
  state_within.y = 9.7;
  state_within.heading = 0.0;
  state_within.linear_speed = 0.0;
  state_within.angular_speed = 0.0;

  EXPECT_TRUE(hold.isAtHoldPosition(state_within, 0.5));

  // Outside Euclidean tolerance (distance > 0.5)
  // sqrt(0.4^2 + 0.4^2) = sqrt(0.32) ≈ 0.566 > 0.5
  RobotState state_out;
  state_out.x = 5.4;
  state_out.y = 9.6;
  state_out.heading = 0.0;
  state_out.linear_speed = 0.0;
  state_out.angular_speed = 0.0;

  EXPECT_FALSE(hold.isAtHoldPosition(state_out, 0.5));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
