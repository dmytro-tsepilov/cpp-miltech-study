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
    IBallisticSolver* solver;       // стратегія балістики
    ITargetProvider*  targets;      // провайдер цілей
    IConfigLoader*    configLoader; // завантажувач конфігурації
    IResultWriter*    resultWriter;

    int currentIdx;                 // лічильник поточної цілі

    DroneConfig droneConfig;        // конфігурація дрона
    AmmoType  ammo;                 // параметри боєприпасу
    AmmoType** bombTypes = nullptr;
    double    ballisticTof;         // time of flight (попередньо обчислений)
    double    hDistBomb;            // horizontal distance (попередньо обчислений)

    // Внутрішній стан симуляції
    SimStep*  simSteps;
    int       currentStep;                   // лічильник кроків симуляції

    float     currentSpeed;         // поточна швидкість
    int       remainingTurningSteps;
    double    currentTime;
    float     acceleration;

public:
    // Конструктор — приймає solver і targets через інтерфейси
    MissionProcessor(IBallisticSolver* s, ITargetProvider* t) {
        solver = s;
        targets = t;
    };

    // Деструктор — звільняє пам'ять
    ~MissionProcessor() {
        delete targets;
    };

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
        solver = s;
    };

    // Отримати поточну позицію дрона (для зовнішнього використання)
    Coord getCurrentPos() const { return simSteps[currentStep].pos; }

    // Отримати поточний час
    double getCurrentTime() const { return currentTime; }

    // Отримати кількість цілей
    int getTargetCount() const;

    int calculateFlow();

private:
    Coord targetInterpolation(const int8_t &targetId, const double &time, const float &arrayTimeStep);
    Coord extrapTarget(int targetId, double currentTime, double dtAhead, float dt);
    double applyLimitedTurn(const SimStep &simStep, const double &maxTurnPerStep, const double &desiredDir);
    bool leadTarget(Coord pos, const int tgtIdx, const double &currentTime,
                  const float &attackSpeed, const double &hDistBomb, const double &ballisticTof, float arrayTimeStep,
                  Coord &firePos, Coord &predict);
    void updateDroneState(const Coord& desiredPos, bool inBombingRange, const Coord& bestPredict);
};
