#pragma once

#include <memory>
#include <cmath>
#include <algorithm>
#include <string>
#include "config/DroneConfig.h"

struct DroneContext {
    // Поточний стан дрона
    Coord   pos{};                  // поточна позиція
    float   direction{0.0f};        // поточний напрямок (радіани)
    float   currentSpeed{0.0f};     // поточна швидкість

    // Цільові параметри
    Coord   startPos{};             // початкова позиція
    double  desiredDir{0.0f};       // бажаний напрямок
    Coord   targetPos{};            // ціль для навігації

    // Конфігурація дрона (посилання)
    const DroneConfig* cfg{nullptr};

    // Внутрішні змінні станів
    int     remainingTurningSteps{0};
    float   acceleration{0.0f};
    double  maxTurnPerStep{0.0f};

    // Лічильники
    int     currentStep{0};
    double  currentTime{0.0};
    bool    shouldFire{false};
};

class IDroneState {
public:
    virtual ~IDroneState() = default;

    // Виконати логіку стану, повернути наступний стан.
    // Якщо стан не змінився - повернути nullptr
    // (головний цикл залишить поточний).
    virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx) = 0;

    virtual const std::string name() const = 0;
    virtual int stateId() const = 0;

protected:
    // Helper для нормалізації кута
    static double normalizeAngle(double angle) {
        angle = std::fmod(angle + M_PI, 2 * M_PI);
        if (angle < 0.0f) angle += 2 * M_PI;
        return angle - M_PI;
    }

    // Helper для обмеження повороту
    static double applyLimitedTurn(const double &currentDir, const double &desiredDir, const double &maxTurnPerStep) {
        double diff = desiredDir - currentDir;
        float direction = currentDir;
        diff = normalizeAngle(diff);

        double turn = std::clamp(diff, -maxTurnPerStep, maxTurnPerStep);
        direction += turn;

        direction = normalizeAngle(direction);

        return direction;
    }
};
