#pragma once

#include <mutex>
#include <atomic>
#include "config/DroneConfig.h"

struct DroneCommand {
    float  direction = 0.0f; // напрямок руху (рад)
    float  speed = 0.0f;     // поточна швидкість (м/с)
    int8_t state = 0;        // режим дрона (для телеметрії/логів)
};

// Телеметрія дрона: знімок поточного стану фізики під м'ютексом.
struct DroneTelemetry {
    Coord  pos{};                    // поточна позиція
    float  speed = 0.0f;             // поточна швидкість
    float  direction = 0.0f;         // поточний напрямок (рад)
    int8_t state = 0;                // поточний режим
    float  timeSecSinceStart = 0.0f; // час останнього оновлення фізики
};

// Фізика дрона у власному потоці
class DronePhysics {
private:
    Coord  pos_{};
    float  direction_ = 0.0f;
    float  speed_ = 0.0f;
    int8_t state_ = 0;
    float  timeSecSinceStart_ = 0.0f;

    float  physicsTimeStep_ = 0.01f;
    float  timeScale_ = 1.0f;

    DroneCommand command_{};

    mutable std::mutex mutex_;
    std::atomic<bool> threadReady_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_{false};

public:
    void init(const Coord &startPos, float initialDir, float physicsTimeStep, float timeScale);

    // Передати команду (записується під м'ютексом).
    void setCommand(const DroneCommand &cmd);

    // Зняти телеметрію (копія під м'ютексом).
    DroneTelemetry getTelemetry() const;

    void run();
    void start();
    void stop();
    bool isThreadReady() const;
};
