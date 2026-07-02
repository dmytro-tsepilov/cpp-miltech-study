#include "mission/DroneState.h"
#include <cmath>
#include <algorithm>
#include <memory>

// ============================================
// StateStopped implementation
// ============================================
std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx) {
    float delta = IDroneState::normalizeAngle(ctx.desiredDir - ctx.direction);

    if (std::fabs(delta) > ctx.cfg->turnThreshold) {
        // Потрібно повернути
        ctx.remainingTurningSteps = static_cast<int>(
            std::ceil(std::fabs(delta) / (ctx.cfg->angularSpeed * ctx.cfg->simTimeStep)));
        return std::make_unique<StateTurning>();
    }

    // Вже вирівняний — починаємо прискорюватися
    ctx.direction = ctx.desiredDir;
    return std::make_unique<StateAccelerating>();
}

const std::string StateStopped::name() const {
    return "Stopped";
}


// ============================================
// StateAccelerating implementation
// ============================================
std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx) {
    float delta = IDroneState::normalizeAngle(ctx.desiredDir - ctx.direction);

    // Перевіряємо, чи потрібен розворот (якщо кут великий і ми вже рухаємось)
    if (std::fabs(delta) > ctx.cfg->turnThreshold * 2.0f &&
        ctx.currentSpeed > ctx.cfg->attackSpeed * 0.1f) {
        return std::make_unique<StateDecelerating>();
    }

    // Обмежений поворот під час прискорення
    ctx.direction = IDroneState::applyLimitedTurn(ctx.direction, ctx.desiredDir, ctx.maxTurnPerStep);

    // Прискорення
    ctx.currentSpeed = std::min(ctx.cfg->attackSpeed,
                                 ctx.currentSpeed + ctx.acceleration * ctx.cfg->simTimeStep);

    // Якщо досягли цільової швидкості — переходимо в Moving
    if (ctx.currentSpeed >= ctx.cfg->attackSpeed - 0.01f) {
        return std::make_unique<StateMoving>();
    }

    return nullptr; // Залишаємось в цьому стані
}

const std::string StateAccelerating::name() const {
    return "Accelerating";
}


// ============================================
// StateDecelerating implementation
// ============================================
std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx) {
    // Гальмування
    ctx.currentSpeed = std::max(0.0f, ctx.currentSpeed - ctx.acceleration * ctx.cfg->simTimeStep);

    if (ctx.currentSpeed < 0.01f) {
        ctx.currentSpeed = 0.0f;

        // Після зупинки перевіряємо кут
        float delta = IDroneState::normalizeAngle(ctx.desiredDir - ctx.direction);
        if (std::fabs(delta) > ctx.cfg->turnThreshold) {
            ctx.remainingTurningSteps = static_cast<int>(
                std::ceil(std::fabs(delta) / (ctx.cfg->angularSpeed * ctx.cfg->simTimeStep)));
            return std::make_unique<StateTurning>();
        }

        // Вирівняний — починаємо прискорюватися
        ctx.direction = ctx.desiredDir;
        return std::make_unique<StateAccelerating>();
    }

    return nullptr; // Залишаємось в цьому стані
}

const std::string StateDecelerating::name() const {
    return "Decelerating";
}


// ============================================
// StateTurning implementation
// ============================================
std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx) {
    // Обертання до цільового напрямку
    ctx.direction = IDroneState::applyLimitedTurn(ctx.direction, ctx.desiredDir, ctx.maxTurnPerStep);

    // Перевіряємо, чи довернулися
    float newDiff = std::fabs(IDroneState::normalizeAngle(ctx.desiredDir - ctx.direction));

    --ctx.remainingTurningSteps;
    if (ctx.remainingTurningSteps <= 0 || newDiff < 0.01f) {
        return std::make_unique<StateAccelerating>();
    }

    return nullptr; // Залишаємось в цьому стані
}

const std::string StateTurning::name() const {
    return "Turning";
}


// ============================================
// StateMoving implementation
// ============================================
std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx) {
    float delta = IDroneState::normalizeAngle(ctx.desiredDir - ctx.direction);

    // Якщо потрібен поворот — гальмуємо
    if (std::fabs(delta) > ctx.cfg->turnThreshold * 2.0f) {
        return std::make_unique<StateDecelerating>();
    }

    // Обмежений поворот під час руху
    ctx.direction = IDroneState::applyLimitedTurn(ctx.direction, ctx.desiredDir, ctx.maxTurnPerStep);

    // Максимальна швидкість
    ctx.currentSpeed = ctx.cfg->attackSpeed;

    return nullptr; // Залишаємось в цьому стані
}

const std::string StateMoving::name() const {
    return "Moving";
}

