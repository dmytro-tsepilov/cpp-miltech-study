#pragma once

#include <mutex>
#include "mission/IDroneStateSource.h"

// UartDroneState — джерело стану дрона для UART-режиму.
//
// На відміну від DronePhysics, воно НЕ інтегрує рух: фізику інтегрує чекер.
//   * getTelemetry() повертає останню телеметрію, отриману від чекера
//     (її треба щокроку оновлювати через setTelemetry());
//   * stepCommand() лише запам'ятовує команду, яку вирішила місія
//     (бажаний курс/швидкість), щоб модуль керування перетворив її на CONTROL.
class UartDroneState : public IDroneStateSource {
private:
    mutable std::mutex mutex_;
    DroneTelemetry telemetry_{};
    DroneCommand   lastCommand_{};

public:
    UartDroneState() = default;
    ~UartDroneState() override = default;

    void init(const Coord& startPos, float initialDir,
              std::unique_ptr<ITimeProvider> timeProvider,
              float simTimeStep, float physicsTimeStep) override;

    DroneTelemetry getTelemetry() const override;
    void stepCommand(const DroneCommand& cmd) override;

    // Оновити стан дрона з телеметрії чекера (викликається щокроку в main).
    void setTelemetry(const DroneTelemetry& tel);

    // Остання команда, яку вирішила місія (бажаний курс/швидкість).
    DroneCommand getLastCommand() const;
};
