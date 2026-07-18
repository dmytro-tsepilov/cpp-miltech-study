#include <atomic>
#include <cmath>
#include "providers/FixedTimeProvider.h"

void FixedTimeProvider::init(float physicsTimeStep, float timeScale)
{
    physicsTimeStep_ = physicsTimeStep > 0.0f ? physicsTimeStep : 0.01f;
    timeScale_ = timeScale > 0.0f ? timeScale : 1.0f;
    currentSimTime_ = 0.0f;
    stepCount_ = 0;
}

float FixedTimeProvider::getPhysicsDeltaTime() const
{
    return physicsTimeStep_;
}

float FixedTimeProvider::getCurrentSimTime() const
{
    return currentSimTime_;
}

void FixedTimeProvider::tick()
{
    if (started_ && !stop_) {
        stepCount_++;
    }
}

std::chrono::milliseconds FixedTimeProvider::getSleepDuration() const
{
    // Цільовий час сну з урахуванням timeScale.
    // Наприклад: physicsTimeStep_ = 0.01s, timeScale_ = 10.0
    // sleepMs = (0.01 / 10.0) * 1000 = 1ms
    float sleepSec = physicsTimeStep_ / timeScale_;
    int sleepMs = static_cast<int>(std::round(sleepSec * 1000.0f));
    
    if (sleepMs < 1) {
        sleepMs = 1;
    }

    return std::chrono::milliseconds(sleepMs);
}

void FixedTimeProvider::start() 
{ 
    started_ = true; 
    stop_ = false;
}

void FixedTimeProvider::stop() 
{ 
    stop_ = true; 
}

bool FixedTimeProvider::isRunning() const 
{ 
    return started_ && !stop_; 
}
