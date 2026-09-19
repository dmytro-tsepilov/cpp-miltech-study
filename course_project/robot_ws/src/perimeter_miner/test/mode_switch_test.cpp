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


#include <gtest/gtest.h>
#include <memory>

#include "perimeter_miner/mode_switch.hpp"

using perimeter_miner::ControlMode;
using perimeter_miner::ModeSwitch;

// Test 1: Default mode is AUTONOMOUS
TEST(ModeSwitchDefaultTest, DefaultMode)
{
  ModeSwitch ms;
  EXPECT_EQ(ms.getCurrentMode(), ControlMode::AUTONOMOUS);
}

// Test 2: Request and apply mode change
TEST(ModeSwitchRequestTest, RequestAndApply)
{
  ModeSwitch ms;

  bool success = ms.requestMode(ControlMode::TELEOP);
  EXPECT_TRUE(success);
  EXPECT_TRUE(ms.isSwitching());

  success = ms.applyRequest();
  EXPECT_TRUE(success);
  EXPECT_EQ(ms.getCurrentMode(), ControlMode::TELEOP);
  EXPECT_FALSE(ms.isSwitching());
}

// Test 3: Operator override
TEST(ModeSwitchOverrideTest, OperatorOverride)
{
  ModeSwitch ms;

  bool overridden = ms.operatorOverride();
  EXPECT_TRUE(overridden);
  EXPECT_EQ(ms.getCurrentMode(), ControlMode::TELEOP);
}

// Test 4: Duplicate operator override
TEST(ModeSwitchOverrideTest, DuplicateOverride)
{
  ModeSwitch ms;

  bool first = ms.operatorOverride();
  EXPECT_TRUE(first);

  bool second = ms.operatorOverride();
  EXPECT_FALSE(second);  // Already in TELEOP
}

// Test 5: Cannot request same mode
TEST(ModeSwitchRequestTest, SameModeRequest)
{
  ModeSwitch ms;

  bool success = ms.requestMode(ControlMode::AUTONOMOUS);
  EXPECT_FALSE(success);  // Already in AUTONOMOUS
}

// Test 6: All mode transitions
TEST(ModeSwitchTransitionsTest, AllTransitions)
{
  ModeSwitch ms;

  // AUTONOMOUS -> TELEOP
  ms.requestMode(ControlMode::TELEOP);
  ms.applyRequest();
  EXPECT_EQ(ms.getCurrentMode(), ControlMode::TELEOP);

  // TELEOP -> HOLD
  ms.requestMode(ControlMode::HOLD);
  ms.applyRequest();
  EXPECT_EQ(ms.getCurrentMode(), ControlMode::HOLD);

  // HOLD -> AUTONOMOUS
  ms.requestMode(ControlMode::AUTONOMOUS);
  ms.applyRequest();
  EXPECT_EQ(ms.getCurrentMode(), ControlMode::AUTONOMOUS);
}

// Test 7: Safety check for autonomous mode
TEST(ModeSwitchSafetyTest, AutonomousSafetyCheck)
{
  ModeSwitch ms;

  // Set a failing safety check
  ms.setAutonomousCheck([]() { return false; });

  // Switch to TELEOP first
  ms.requestMode(ControlMode::TELEOP);
  ms.applyRequest();

  // Try to switch back to AUTONOMOUS (should fail)
  bool success = ms.requestMode(ControlMode::AUTONOMOUS);
  // Request may succeed, but apply should fail
  if (success) {
    success = ms.applyRequest();
    EXPECT_FALSE(success);
  }
}

// Test 8: Last message tracking
TEST(ModeSwitchMessagesTest, LastMessageTracking)
{
  ModeSwitch ms;

  ms.requestMode(ControlMode::TELEOP);
  std::string msg_before = ms.getLastMessage();
  EXPECT_FALSE(msg_before.empty());

  ms.applyRequest();
  std::string msg_after = ms.getLastMessage();
  EXPECT_FALSE(msg_after.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
