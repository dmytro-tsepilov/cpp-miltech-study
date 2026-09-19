#pragma once

#include "perimeter_miner/perimeter_config.hpp"

namespace perimeter_miner {

/// Hold position controller (maintains position when stopped)
class HoldController {
public:
    HoldController() = default;
    
    /// Set hold position
    void setHoldPosition(double x, double y, double heading);
    
    /// Compute hold command based on current state
    MoveCommand compute(const RobotState& state);
    
    /// Check if robot is at hold position
    bool isAtHoldPosition(const RobotState& state, double tolerance = 0.5) const;

private:
    double hold_x_ = 0.0;
    double hold_y_ = 0.0;
    double hold_heading_ = 0.0;
    
    // PID gains for position hold
    double pos_kp_ = 2.0;
    double pos_ki_ = 0.5;
    double pos_kd_ = 0.3;
    
    double heading_kp_ = 3.0;
    double heading_ki_ = 0.5;
    double heading_kd_ = 0.2;
    
    // Anti-windup limits
    double max_linear_ = 1.0;
    double max_angular_ = 1.0;
    
    // Integral terms
    double pos_integral_x_ = 0.0;
    double pos_integral_y_ = 0.0;
    double heading_integral_ = 0.0;
    
    // Integral limits
    double max_pos_integral_ = 5.0;
    double max_heading_integral_ = 3.0;
    
    // Normalize angle
    static double normalizeAngle(double angle);
    
    // Angle difference
    static double angleDiff(double from, double to);
};

} // namespace perimeter_miner
