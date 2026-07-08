#pragma once
// UartStepDriver — реалізація IStepDriver для UART/GPIO режиму (HW22).
//
// Інкапсулює всю обв'язку вводу-виводу навколо кроку місії:
//   * пейсинг по кадрах телеметрії чекера (кожен кадр обробляється рівно раз);
//   * подача телеметрії від чекера у джерело стану дрона (UartDroneState);
//   * відправка нормованої команди CONTROL назад чекеру по UART;
//   * імпульс DROP на GPIO у момент скиду.
//
// Завдяки цьому MissionProcessor лишається чистим від деталей UART/GPIO.

#ifndef UART_STEP_DRIVER_H
#define UART_STEP_DRIVER_H

#include <cstdint>
#include "mission/IStepDriver.h"
#include "mission/IDroneStateSource.h" // DroneTelemetry

class IUartLink;
class IDroneGpioController;
class IMissionCommandSource;
class IUartTelemetryProvider;
class UartDroneState;

class UartStepDriver : public IStepDriver {
public:
    UartStepDriver(IUartLink* uart,
                   IDroneGpioController* gpio,
                   IMissionCommandSource* cmdSource,
                   IUartTelemetryProvider* telProvider,
                   UartDroneState* droneState,
                   double maxTurnPerStep,
                   float accelPerStep,
                   int maxSteps = 10000);

    // Заблокуватися до нового кадру телеметрії чекера (пейсинг по t_ms).
    bool waitNextTick() override;

    // Зчитати телеметрію чекера і подати її у джерело стану дрона.
    void beforeStep() override;

    // Перетворити рішення місії на CONTROL і надіслати по UART.
    void afterStep() override;

    // Надіслати нульове керування і імпульс DROP на GPIO.
    void onDrop() override;

private:
    IUartLink*              uart_;
    IDroneGpioController*    gpio_;
    IMissionCommandSource*   cmdSource_;
    IUartTelemetryProvider*  telProvider_;
    UartDroneState*          droneState_;

    double maxTurnPerStep_;
    float  accelPerStep_;
    int    maxSteps_;

    int      step_ = 0;
    uint32_t lastTelemetryMs_ = 0;
    bool     firstFrame_ = true;
    DroneTelemetry dt_{}; // кадр телеметрії, підготовлений у beforeStep()
};

#endif // UART_STEP_DRIVER_H
