#pragma once

#include "config/DroneConfig.h"
// ============ ITargetProvider Interface ============

class ITargetProvider {
public:
    virtual ~ITargetProvider() = default;
    virtual bool load() = 0;
    virtual int getTargetCount() = 0;
    virtual int getTimeSteps() = 0;
    virtual Target getTarget(int index) = 0;

    // ---- Багатопоточність ----
    virtual void setTimings(float arrayTimeStep, float timeScale) { (void)arrayTimeStep; (void)timeScale; }
    virtual void run() {}
    virtual void start() {}
    virtual void stop() {}
    virtual bool isThreadReady() { return true; }
};
