#pragma once
// IMissionCommandSource — generates CONTROL commands from mission logic

#ifndef I_MISSION_COMMAND_SOURCE_H
#define I_MISSION_COMMAND_SOURCE_H

#include <atomic>
#include "protocol/drone_link.h"

class IMissionCommandSource {
public:
    virtual ~IMissionCommandSource() = default;

    // Generate CONTROL command from current telemetry and target data.
    // accel: normalized acceleration along heading [-1..1]
    // turnRate: normalized turn rate [-1..1]
    virtual void generateCommand(const dlink::Telemetry& tel,
                                 const dlink::TargetPos& target,
                                 float& accel,
                                 float& turnRate) = 0;

    // Check if bomb should be dropped
    virtual bool shouldDrop() const = 0;

    // Reset drop flag after pulse sent
    virtual void resetDrop() = 0;

    // Initialize with mission parameters
    virtual void init(float attackSpeed, float maxTurnRate) = 0;

    // Check if command source is ready
    virtual bool isReady() const = 0;
};

#endif // I_MISSION_COMMAND_SOURCE_H
