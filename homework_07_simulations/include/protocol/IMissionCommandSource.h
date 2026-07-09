#pragma once
// IMissionCommandSource — модуль керування дроном.
// Перетворює рішення місії (бажаний курс/швидкість) на нормовані команди
// UART CONTROL (accel, turnRate) з урахуванням поточної телеметрії чекера.

#ifndef I_MISSION_COMMAND_SOURCE_H
#define I_MISSION_COMMAND_SOURCE_H

#include "mission/IDroneStateSource.h"

class IMissionCommandSource {
public:
    virtual ~IMissionCommandSource() = default;

    // Перетворити команду місії на нормовані accel/turnRate у діапазоні [-1..1].
    //   cmd            — бажаний курс/швидкість, які вирішила місія;
    //   tel            — поточний стан дрона (від чекера);
    //   maxTurnPerStep — макс. поворот за крок місії (рад), для нормування turnRate;
    //   accelPerStep   — макс. зміна швидкості за крок місії (м/с), для нормування accel.
    virtual void computeControl(const DroneCommand& cmd,
                                const DroneTelemetry& tel,
                                double maxTurnPerStep,
                                float accelPerStep,
                                float& accel,
                                float& turnRate) = 0;
};

#endif // I_MISSION_COMMAND_SOURCE_H
