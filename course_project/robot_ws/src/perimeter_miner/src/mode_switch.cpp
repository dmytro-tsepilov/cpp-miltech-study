#include "perimeter_miner/mode_switch.hpp"

namespace perimeter_miner {

bool ModeSwitch::requestMode(ControlMode requested) {
    if (requested == current_mode_) {
        last_message_ = "Already in mode: " + std::string(controlModeToString(requested));
        return false;
    }
    
    // Validate the switch
    if (!validateModeSwitch(requested)) {
        last_message_ = "Cannot switch to " + std::string(controlModeToString(requested)) + " - conditions not met";
        return false;
    }
    
    pending_mode_ = requested;
    switching_ = true;
    last_message_ = "Mode switch requested: " + std::string(controlModeToString(requested));
    return true;
}

bool ModeSwitch::applyRequest() {
    if (!switching_) {
        last_message_ = "No pending mode switch";
        return false;
    }
    
    // Check safety conditions
    if (pending_mode_ == ControlMode::AUTONOMOUS) {
        if (autonomous_check_ && !autonomous_check_()) {
            last_message_ = "Cannot return to autonomous - safety check failed";
            switching_ = false;
            pending_mode_ = current_mode_;
            return false;
        }
    }
    
    // Apply the switch
    updateState(pending_mode_);
    switching_ = false;
    last_message_ = "Mode switched to: " + std::string(controlModeToString(current_mode_));
    return true;
}

bool ModeSwitch::operatorOverride() {
    // Operator override always succeeds (priority)
    if (current_mode_ == ControlMode::TELEOP) {
        last_message_ = "Already in TELEOP mode";
        return false;
    }
    
    updateState(ControlMode::TELEOP);
    switching_ = false;
    pending_mode_ = current_mode_;
    last_message_ = "OPERATOR OVERRIDE - switched to TELEOP";
    return true;
}

bool ModeSwitch::validateModeSwitch(ControlMode target) const {
    switch (target) {
        case ControlMode::AUTONOMOUS:
            // Can always switch to autonomous (safety check done in applyRequest)
            return true;
            
        case ControlMode::TELEOP:
            // Can always switch to teleop (operator has priority)
            return true;
            
        case ControlMode::HOLD:
            // Can always hold position
            return true;
            
        default:
            return false;
    }
}

void ModeSwitch::updateState(ControlMode new_mode) {
    current_mode_ = new_mode;
    pending_mode_ = new_mode;
}

} // namespace perimeter_miner
