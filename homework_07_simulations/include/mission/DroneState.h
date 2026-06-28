#pragma once

#include "interfaces/IDroneState.h"

// forward declarations for state transitions
class StateAccelerating;
class StateDecelerating;
class StateTurning;
class StateMoving;

// ============================================
// Стан STOPPED: дрон зупинений
// Перевіряє кут → переходить в Turning або Accelerating
// ============================================
class StateStopped : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const std::string name() const override;
    int stateId() const override { return 0; }
};

// ============================================
// Стан ACCELERATING: дрон розганяється
// Набір швидкості → Moving (або Decelerating якщо потрібен розворот)
// ============================================
class StateAccelerating : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const std::string name() const override;
    int stateId() const override { return 1; }
};

// ============================================
// Стан DECELERATING: дрон гальмує
// Гальмування → Stopped (швидкість = 0)
// ============================================
class StateDecelerating : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    int stateId() const override { return 2; }
    const std::string name() const override;
};

// ============================================
// Стан TURNING: дрон обертається на місці
// Обертання → Accelerating (коли довернувся)
// ============================================
class StateTurning : public IDroneState {
public:
    int stateId() const override { return 3; }
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const std::string name() const override;
};

// ============================================
// Стан MOVING: дрон рухається на макс. швидкості
// Рух → Decelerating (якщо потрібен розворот)
// ============================================
class StateMoving : public IDroneState {
    int stateId() const override { return 4; }
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const std::string name() const override;
};
