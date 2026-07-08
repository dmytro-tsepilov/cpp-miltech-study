#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>

#include "mission/DronePhysics.h"
#include "interfaces/ITimeProvider.h"

void DronePhysics::init(const Coord &startPos, float initialDir,
                        std::unique_ptr<ITimeProvider> timeProvider,
                        float simTimeStep, float physicsTimeStep)
{
    pos_ = startPos;
    direction_ = initialDir;
    speed_ = 0.0f;
    state_ = 0;
    timeSecSinceStart_ = 0.0f;

    timeProvider_ = std::move(timeProvider);

    simTimeStep_ = simTimeStep > 0.0f ? simTimeStep : 0.1f;
    physicsTimeStep_ = physicsTimeStep > 0.0f ? physicsTimeStep : 0.01f;

    command_.direction = initialDir;
    command_.speed = 0.0f;
    command_.state = 0;
}

void DronePhysics::setCommand(const DroneCommand &cmd)
{
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = cmd;
}

void DronePhysics::stepCommand(const DroneCommand &cmd)
{
    std::unique_lock<std::mutex> lock(mutex_);

    command_ = cmd;
    direction_ = cmd.direction;
    speed_ = cmd.speed;
    state_ = cmd.state;

    // Request the physics thread to integrate exactly one mission step and wait.
    stepRequested_ = true;
    stepCompleted_ = false;
    cv_.notify_all();
    cv_.wait(lock, [this] { return stepCompleted_ || stop_; });
}

DroneTelemetry DronePhysics::getTelemetry() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    DroneTelemetry tel;
    tel.pos = pos_;
    tel.speed = speed_;
    tel.direction = direction_;
    tel.state = state_;
    tel.timeSecSinceStart = timeSecSinceStart_;
    return tel;
}

void DronePhysics::run()
{
    threadReady_ = true;

    while (!started_ && !stop_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (timeProvider_) {
        timeProvider_->start();
    }

    // Deterministic number of sub-steps per mission step.
    const int subSteps = std::max(1, static_cast<int>(std::lround(simTimeStep_ / physicsTimeStep_)));

    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_) {
        // Wait for a step request from the mission (or stop).
        cv_.wait(lock, [this] { return stepRequested_ || stop_; });
        if (stop_) {
            break;
        }

        // We integrate EXACTLY subSteps fixed physicsTimeStep_ sub-steps. 
        // The number of sub-steps does not depend on the OS scheduler and sleep jitter,
        // so the movement per mission step is always = speed * simTimeStep_.
        // timeSecSinceStart_ is incremented by the same fixed steps —
        // fully synchronized with the logical mission time (simTimeStep per step).
        for (int i = 0; i < subSteps; ++i) {
            pos_.x += std::cos(direction_) * speed_ * physicsTimeStep_;
            pos_.y += std::sin(direction_) * speed_ * physicsTimeStep_;
            timeSecSinceStart_ += physicsTimeStep_;
        }

        stepRequested_ = false;
        stepCompleted_ = true;
        cv_.notify_all();
    }

    if (timeProvider_) {
        timeProvider_->stop();
    }
}

void DronePhysics::start()
{
    started_ = true;
}

void DronePhysics::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }

    cv_.notify_all();
}

bool DronePhysics::isThreadReady() const
{
    return threadReady_;
}
