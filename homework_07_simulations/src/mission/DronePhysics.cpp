#include <cmath>
#include <thread>
#include <chrono>

#include "mission/DronePhysics.h"

void DronePhysics::init(const Coord &startPos, float initialDir, float physicsTimeStep, float timeScale)
{
    pos_ = startPos;
    direction_ = initialDir;
    speed_ = 0.0f;
    state_ = 0;
    timeSecSinceStart_ = 0.0f;
    physicsTimeStep_ = physicsTimeStep > 0.0f ? physicsTimeStep : 0.01f;
    timeScale_ = timeScale > 0.0f ? timeScale : 1.0f;

    command_.direction = initialDir;
    command_.speed = 0.0f;
    command_.state = 0;
}

void DronePhysics::setCommand(const DroneCommand &cmd)
{
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = cmd;
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

    auto prevTime = std::chrono::steady_clock::now();
    while (!stop_) {
        // Інтегруємо за фактично проведеним часом (масштабованим timeScale),
        // а не за кількістю ітерацій — щоб джиттер планувальника потоків не
        // спотворював пройдену відстань і похідне прискорення.
        auto now = std::chrono::steady_clock::now();
        float simDt = std::chrono::duration<float>(now - prevTime).count() * timeScale_;
        prevTime = now;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            // Виконуємо поточну команду: приймаємо напрямок/швидкість,
            // інтегруємо лише позицію.
            direction_ = command_.direction;
            speed_ = command_.speed;
            state_ = command_.state;

            pos_.x += std::cos(direction_) * speed_ * simDt;
            pos_.y += std::sin(direction_) * speed_ * simDt;
            timeSecSinceStart_ += simDt;
        }

        std::this_thread::sleep_for(std::chrono::duration<float>(physicsTimeStep_ / timeScale_));
    }
}

void DronePhysics::start()
{
    started_ = true;
}

void DronePhysics::stop()
{
    stop_ = true;
}

bool DronePhysics::isThreadReady() const
{
    return threadReady_;
}
