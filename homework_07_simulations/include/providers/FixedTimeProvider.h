#pragma once

#include <atomic>
#include "interfaces/ITimeProvider.h"

// Fixed time provider with deterministic steps.
// timeSecSinceStart is incremented by fixed physicsTimeStep_ steps,
// eliminating acceleration spikes due to real-time clock jitter.
class FixedTimeProvider : public ITimeProvider {
private:
    float physicsTimeStep_ = 0.01f;   // фіксований крок фізики
    float timeScale_ = 1.0f;          // прискорення часу симуляції
    float currentSimTime_ = 0.0f;     // поточний симульований час
    int stepCount_ = 0;               // лічильник кроків для моніторингу
    
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_{false};

public:
    void init(float physicsTimeStep, float timeScale) override;
    float getPhysicsDeltaTime() const override;
    float getCurrentSimTime() const override;
    void tick() override;
    std::chrono::milliseconds getSleepDuration() const override;
    void start() override;
    void stop() override;
    bool isRunning() const override;
    
    int getStepCount() const { return stepCount_; }
    float getPhysicsTimeStep() const { return physicsTimeStep_; }
    float getTimeScale() const { return timeScale_; }
};
