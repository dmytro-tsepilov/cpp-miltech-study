#pragma once

#include <mutex>
#include <atomic>
#include <memory>
#include <condition_variable>
#include "config/DroneConfig.h"
#include "mission/IDroneStateSource.h"

class ITimeProvider;

// Фізика дрона у власному потоці
// Використовує ITimeProvider для детермінованого інтегрування фізики
// з фіксованими кроками, що усуває спайки прискорення через jitter OS.
class DronePhysics : public IDroneStateSource {
private:
    Coord  pos_{};
    float  direction_ = 0.0f;
    float  speed_ = 0.0f;
    int8_t state_ = 0;
    float  timeSecSinceStart_ = 0.0f;

    // Фіксовані кроки: один крок місії = simTimeStep_, який інтегрується
    // рівно (simTimeStep_ / physicsTimeStep_) під-кроками physicsTimeStep_.
    float  simTimeStep_ = 0.1f;
    float  physicsTimeStep_ = 0.01f;

    // TimeProvider для детермінованого часу
    std::unique_ptr<ITimeProvider> timeProvider_;

    DroneCommand command_{};

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool stepRequested_ = false;
    bool stepCompleted_ = false;
    std::atomic<bool> threadReady_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_{false};

public:
    void init(const Coord &startPos, float initialDir,
              std::unique_ptr<ITimeProvider> timeProvider,
              float simTimeStep, float physicsTimeStep) override;

    // Передати команду (записується під м'ютексом).
    void setCommand(const DroneCommand &cmd);

    // Синхронно проінтегрувати РІВНО один крок місії (simTimeStep_) фіксованими
    // під-кроками physicsTimeStep_ і дочекатися завершення. Робить кількість
    // під-кроків детермінованою — усуває спайки прискорення від sleep jitter.
    void stepCommand(const DroneCommand &cmd) override;

    // Зняти телеметрію (копія під м'ютексом).
    DroneTelemetry getTelemetry() const override;

    void run();
    void start();
    void stop();
    bool isThreadReady() const;
};
