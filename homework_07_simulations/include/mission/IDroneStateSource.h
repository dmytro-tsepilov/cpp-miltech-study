#pragma once

#include <memory>
#include "config/DroneConfig.h"

class ITimeProvider;

// Команда для джерела стану дрона: бажаний курс і швидкість, які вирішила місія.
struct DroneCommand {
    float  direction = 0.0f; // напрямок руху (рад)
    float  speed = 0.0f;     // поточна швидкість (м/с)
    int8_t state = 0;        // режим дрона (для телеметрії/логів)
};

// Телеметрія дрона: знімок поточного стану.
struct DroneTelemetry {
    Coord  pos{};                    // поточна позиція
    float  speed = 0.0f;             // поточна швидкість
    float  direction = 0.0f;         // поточний напрямок (рад)
    int8_t state = 0;                // поточний режим
    float  timeSecSinceStart = 0.0f; // час останнього оновлення
};

// IDroneStateSource — абстракція джерела стану дрона для MissionProcessor.
//
// Дві реалізації:
//   * DronePhysics   — локально інтегрує рух (файловий/HTTP режим);
//   * UartDroneState — віддає стан, отриманий від чекера по UART, і лише
//                      запам'ятовує команду місії (інтегрує чекер).
//
// Це дозволяє MissionProcessor використовувати ту саму логіку наведення
// незалежно від того, хто інтегрує фізику — ми чи чекер.
class IDroneStateSource {
public:
    virtual ~IDroneStateSource() = default;

    // Ініціалізація початкового стану.
    virtual void init(const Coord& startPos, float initialDir,
                      std::unique_ptr<ITimeProvider> timeProvider,
                      float simTimeStep, float physicsTimeStep) = 0;

    // Зняти телеметрію (поточний стан дрона).
    virtual DroneTelemetry getTelemetry() const = 0;

    // Передати команду місії (бажаний курс/швидкість).
    virtual void stepCommand(const DroneCommand& cmd) = 0;
};
