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

#include <functional>
#include <mutex>
#include <string>

#include "perimeter_miner/perimeter_config.hpp"

namespace perimeter_miner
{

/// Mode switch controller with operator override capability
class ModeSwitch
{
public:
ModeSwitch() = default;

/// Request mode change (can be denied based on conditions)
bool requestMode(ControlMode requested);

/// Apply pending mode change (check safety conditions)
bool applyRequest();

/// Operator override - immediate switch to TELEOP (priority)
bool operatorOverride();

/// Get current mode
ControlMode getCurrentMode() const {
  return current_mode_;
}

/// Get pending mode
ControlMode getPendingMode() const {
  return pending_mode_;
}

/// Check if mode switching is in progress
bool isSwitching() const {
  return switching_;
}

/// Check if operator is active
bool isOperatorActive() const
{
  return current_mode_ == ControlMode::TELEOP;
}

/// Get last switch message
const std::string &getLastMessage() const {
  return last_message_;
}

/// Set conditions for mode switching
using ModeCheck = std::function<bool ()>;
void setAutonomousCheck(ModeCheck check) {
  autonomous_check_ = check;
}
void setTeleopCheck(ModeCheck check) {
  teleop_check_ = check;
}

private:
ControlMode current_mode_ = ControlMode::AUTONOMOUS;
ControlMode pending_mode_ = ControlMode::AUTONOMOUS;
bool switching_ = false;
std::string last_message_;

// Safety checks
ModeCheck autonomous_check_;
ModeCheck teleop_check_;

// Validate mode switch request
bool validateModeSwitch(ControlMode target) const;

// Update internal state
void updateState(ControlMode new_mode);
};

}  // namespace perimeter_miner
