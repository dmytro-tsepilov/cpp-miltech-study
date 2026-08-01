#include "mission/UartDroneState.h"
#include "interfaces/ITimeProvider.h"

void UartDroneState::init(const Coord& startPos, float initialDir,
                          std::unique_ptr<ITimeProvider> timeProvider,
                          float simTimeStep, float physicsTimeStep)
{
    (void)timeProvider;     // фізику інтегрує чекер — таймер не потрібен
    (void)simTimeStep;
    (void)physicsTimeStep;

    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_ = DroneTelemetry{};
    telemetry_.pos = startPos;
    telemetry_.direction = initialDir;
    telemetry_.speed = 0.0f;
    telemetry_.state = 0;

    lastCommand_ = DroneCommand{};
    lastCommand_.direction = initialDir;
}

DroneTelemetry UartDroneState::getTelemetry() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return telemetry_;
}

void UartDroneState::stepCommand(const DroneCommand& cmd)
{
    // Не інтегруємо — лише запам'ятовуємо рішення місії.
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = cmd;
}

void UartDroneState::setTelemetry(const DroneTelemetry& tel)
{
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_ = tel;
}

DroneCommand UartDroneState::getLastCommand() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return lastCommand_;
}
