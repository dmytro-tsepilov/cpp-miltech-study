#include "protocol/IMissionCommandSource.h"

#include <memory>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

// MissionCommandSource — generates CONTROL commands from mission logic
// This bridges the existing MissionProcessor state machine to UART CONTROL packets.
class MissionCommandSource : public IMissionCommandSource {
private:
    float attackSpeed_ = 0.0f;
    float maxTurnRate_ = 0.0f;

    // Current command output
    float currentAccel_ = 0.0f;
    float currentTurnRate_ = 0.0f;

    // Drop state
    std::atomic<bool> dropRequested_{false};

    // Mission state (mirrored from MissionProcessor)
    int currentState_ = 0;  // 0=STOPPED, 1=ACCELERATING, 2=DECELERATING, 3=TURNING, 4=MOVING
    float currentSpeed_ = 0.0f;
    double currentDir_ = 0.0;
    bool atFirePoint_ = false;

    // Angle normalization helper
    static double normalizeAngle(double angle) {
        angle = std::fmod(angle + M_PI, 2 * M_PI);
        if (angle < 0) angle += 2 * M_PI;
        return angle - M_PI;
    }

public:
    MissionCommandSource() = default;
    ~MissionCommandSource() override = default;

    void init(float attackSpeed, float maxTurnRate) override {
        attackSpeed_ = attackSpeed;
        maxTurnRate_ = maxTurnRate;
        currentAccel_ = 0.0f;
        currentTurnRate_ = 0.0f;
        dropRequested_ = false;
        currentState_ = 0;
        currentSpeed_ = 0.0f;
        currentDir_ = 0.0;
        atFirePoint_ = false;
    }

    void generateCommand(const dlink::Telemetry& tel,
                         const dlink::TargetPos& target,
                         float& accel,
                         float& turnRate) override {
        // Update internal state from telemetry
        currentSpeed_ = tel.speed;
        currentDir_ = tel.dir;

        // Calculate distance to target
        double dx = static_cast<double>(target.x) - static_cast<double>(tel.x);
        double dy = static_cast<double>(target.y) - static_cast<double>(tel.y);
        double distToTarget = std::hypot(dx, dy);

        // Calculate desired direction to target
        double desiredDir = std::atan2(dy, dx);
        double angleDiff = normalizeAngle(desiredDir - currentDir_);

        // Determine turn rate based on angle difference
        // Normalize by maxTurnRate_ to get [-1, 1] range
        float turnCmd = 0.0f;
        if (std::abs(angleDiff) > 1e-6) {
            turnCmd = static_cast<float>(std::clamp(angleDiff / static_cast<double>(maxTurnRate_), -1.0, 1.0));
        }

        // Determine acceleration based on current speed vs target speed
        float accelCmd = 0.0f;
        if (currentSpeed_ < attackSpeed_ * 0.9f) {
            accelCmd = 1.0f;  // Accelerate
        } else if (currentSpeed_ > attackSpeed_ * 1.1f) {
            accelCmd = -1.0f;  // Decelerate
        } else {
            accelCmd = 0.0f;  // Maintain speed
        }

        // Check if we're at the fire point (close to target and oriented correctly)
        if (distToTarget < 50.0f && std::abs(angleDiff) < 0.1f) {
            atFirePoint_ = true;
            dropRequested_ = true;
            accelCmd = 0.0f;
            turnCmd = 0.0f;
        } else {
            atFirePoint_ = false;
        }

        currentAccel_ = accelCmd;
        currentTurnRate_ = turnCmd;
        currentState_ = atFirePoint_ ? 4 : (currentSpeed_ < attackSpeed_ * 0.9f ? 1 : 4);

        accel = accelCmd;
        turnRate = turnCmd;

        // Debug output
        if (atFirePoint_) {
            std::cout << "[CmdSrc] *** FIRE POINT REACHED! dist=" << distToTarget
                      << " angleDiff=" << angleDiff << " ***" << std::endl;
        }
    }

    bool shouldDrop() const override {
        return dropRequested_.load();
    }

    void resetDrop() override {
        dropRequested_ = false;
    }

    bool isReady() const override {
        return attackSpeed_ > 0 && maxTurnRate_ > 0;
    }
};

// Factory function
std::unique_ptr<IMissionCommandSource> createMissionCommandSource() {
    return std::make_unique<MissionCommandSource>();
}
