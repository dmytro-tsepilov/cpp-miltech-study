// Copyright 2026 Perimeter Miner Project
// SPDX-License-Identifier: MIT

#pragma once

#include <functional>
#include <mutex>
#include <string>

#include "perimeter_miner/perimeter_config.hpp"

namespace perimeter_miner {

/// Mode switch controller with operator override capability
class ModeSwitch {
public:
    ModeSwitch() = default;

    /// Request mode change (can be denied based on conditions)
    bool requestMode(ControlMode requested);

    /// Apply pending mode change (check safety conditions)
    bool applyRequest();

    /// Operator override - immediate switch to TELEOP (priority)
    bool operatorOverride();

    /// Get current mode
    ControlMode getCurrentMode() const { return current_mode_; }

    /// Get pending mode
    ControlMode getPendingMode() const { return pending_mode_; }

    /// Check if mode switching is in progress
    bool isSwitching() const { return switching_; }

    /// Check if operator is active
    bool isOperatorActive() const {
        return current_mode_ == ControlMode::TELEOP;
    }

    /// Get last switch message
    const std::string& getLastMessage() const { return last_message_; }

    /// Set conditions for mode switching
    using ModeCheck = std::function<bool()>;
    void setAutonomousCheck(ModeCheck check) { autonomous_check_ = check; }
    void setTeleopCheck(ModeCheck check) { teleop_check_ = check; }

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
