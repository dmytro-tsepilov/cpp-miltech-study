#pragma once

#include <chrono>

// Interface for time provider in simulation.
// Allows abstraction of time logic from physics and mission,
// supporting both fixed time steps (for deterministic physics)
// and real-time clock (for real-time mode).

class ITimeProvider {
public:
    virtual ~ITimeProvider() = default;

    virtual void init(float physicsTimeStep, float timeScale) = 0;

    virtual float getPhysicsDeltaTime() const = 0;
    virtual float getCurrentSimTime() const = 0;

    virtual void tick() = 0;

    virtual std::chrono::milliseconds getSleepDuration() const = 0;

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual bool isRunning() const = 0;
};
