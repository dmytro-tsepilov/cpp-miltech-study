#pragma once
#include "target/TargetLoader.h"
#include "result/ResultWriter.h"
#include "ballistic/BallisticSolver.h"
#include "drone/ConfigLoader.h"
#include "drone/DroneConfig.h"
#include "common/SimStep.h"

const int MAX_STEPS = 10000;

enum DroneState : int8_t
{
    STOPPED      = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING      = 3,
    MOVING       = 4,
};

class MissionProcessor {
private:
    IBallisticSolver* solver_;       // стратегія балістики
    ITargetProvider*  targets_;      // провайдер цілей
    IConfigLoader*    configLoader_; // завантажувач конфігурації
    IResultWriter*    resultWriter_;

    int currentIdx_;                 // лічильник поточної цілі

    DroneConfig droneConfig_;        // конфігурація дрона
    AmmoType  ammo_;                 // параметри боєприпасу
    AmmoType** bombTypes_ = nullptr;
    double    ballisticTof_;         // time of flight (попередньо обчислений)
    double    hDistBomb_;            // horizontal distance (попередньо обчислений)
    int       maxTurnPerStep_;       // максимальний поворот за крок (попередньо обчислений)

    // Внутрішній стан симуляції
    SimStep*  simSteps_;
    int       currentStep_;          // лічильник кроків симуляції
    int       targetCount_;          // кількість цілей

    float     currentSpeed_;         // поточна швидкість
    double    currentTime_;
    float     acceleration_;

public:
    // Конструктор — приймає solver і targets через інтерфейси
    MissionProcessor(IBallisticSolver* s, ITargetProvider* t) {
        solver_ = s;
        targets_ = t;
    };

    // Деструктор — звільняє пам'ять
    ~MissionProcessor() {};

    // Ініціалізація: завантажити конфіг через IConfigLoader, підготувати дані
    bool init(IConfigLoader* loader, IResultWriter* resultWriter);

    // Перевірити, чи є ще необроблені цілі
    bool hasNext();

    // Обробити наступну ціль: взяти дані з targets, обчислити через solver, повернути DropPoint
    SimStep step();

    // Почати ітерацію спочатку
    void reset();

    // Підмінити solver на льоту (Стратегія)
    void changeSolver(IBallisticSolver* s) {
        solver_ = s;
    };

    // Отримати поточну позицію дрона (для зовнішнього використання)
    Coord getCurrentPos() const { return simSteps_[currentStep_].pos; }

    // Отримати поточний час
    double getCurrentTime() const { return currentTime_; }

    // Отримати кількість цілей
    int getTargetCount() const { return targets_->getTargetCount(); };

    int calculateFlow();

private:
    Coord targetInterpolation(const int8_t &targetId, const double &time, const float &arrayTimeStep);
    Coord extrapTarget(int targetId, double currentTime, double dtAhead, float dt);
    double applyLimitedTurn(const SimStep &simStep, const double &maxTurnPerStep, const double &desiredDir);
    bool leadTarget(Coord pos, const int tgtIdx, const double &currentTime,
                  const float &attackSpeed, const float &arrayTimeStep,
                  Coord &firePos, Coord &predict);
    void initDroneConstants();
    int detectBestTarget(SimStep &simStep, const double &currentTime, const float &currentSpeed, 
                   const int &remainingTurningSteps, int &bestTargetId, Coord &bestPredict);
};
