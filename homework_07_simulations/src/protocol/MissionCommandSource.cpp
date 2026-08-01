#include "protocol/IMissionCommandSource.h"

#include <memory>
#include <cmath>
#include <algorithm>

// MissionCommandSource — модуль керування дроном для UART-режиму.
//
// Це ТОНКИЙ конвертер: усе наведення (упередження цілі, балістична точка
// скиду, стейт-машина) уже пораховане в MissionProcessor. Тут ми лише
// перетворюємо рішення місії (бажаний курс/швидкість) на нормовані команди
// CONTROL, які чекер множить на фізичні ліміти дрона (maxTurnRate, maxAccel).
class MissionCommandSource : public IMissionCommandSource {
private:
    // Нормалізація кута до діапазону [-pi, pi]
    static double normalizeAngle(double angle) {
        angle = std::fmod(angle + M_PI, 2 * M_PI);
        if (angle < 0) angle += 2 * M_PI;
        return angle - M_PI;
    }

public:
    MissionCommandSource() = default;
    ~MissionCommandSource() override = default;

    void computeControl(const DroneCommand& cmd,
                        const DroneTelemetry& tel,
                        double maxTurnPerStep,
                        float accelPerStep,
                        float& accel,
                        float& turnRate) override
    {
        // --- turnRate ---
        // cmd.direction — бажаний курс (обмежений місією максимум на maxTurnPerStep
        // від поточного). Різниця до реального курсу дрона, нормована на
        // maxTurnPerStep, дає [-1..1]: +1 = поворот вліво (CCW), -1 = вправо.
        double angleDiff = normalizeAngle(static_cast<double>(cmd.direction) - static_cast<double>(tel.direction));
        double turn = 0.0;
        if (maxTurnPerStep > 1e-9) {
            turn = std::clamp(angleDiff / maxTurnPerStep, -1.0, 1.0);
        } else {
            turn = std::clamp(angleDiff, -1.0, 1.0);
        }
        turnRate = static_cast<float>(turn);

        // --- accel ---
        // Різниця бажаної та поточної швидкості, нормована на макс. зміну за крок,
        // дає [-1..1]: +1 = повний газ, -1 = гальмо.
        float speedDiff = cmd.speed - tel.speed;
        if (accelPerStep > 1e-6f) {
            accel = std::clamp(speedDiff / accelPerStep, -1.0f, 1.0f);
        } else {
            accel = std::clamp(speedDiff, -1.0f, 1.0f);
        }
    }
};

// Factory function
std::unique_ptr<IMissionCommandSource> createMissionCommandSource() {
    return std::make_unique<MissionCommandSource>();
}
