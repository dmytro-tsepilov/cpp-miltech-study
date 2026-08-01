#pragma once
#include <vector>
#include <unordered_map>
#include <memory>
#include <atomic>
#include "common/SimStep.h"
#include "interfaces/IResultWriter.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "mission/DroneState.h"
#include "mission/IDroneStateSource.h"
#include "interfaces/ITimeProvider.h"

class ITimeProvider;
class IStepDriver;

const int MAX_STEPS = 10000;

class MissionProcessor {
private:
    std::unique_ptr<IBallisticSolver> solver_;       // стратегія балістики
    std::unique_ptr<ITargetProvider>  targets_;      // провайдер цілей
    std::unique_ptr<IConfigLoader>    configLoader_; // завантажувач конфігурації
    std::unique_ptr<IResultWriter>    resultWriter_;

    DroneConfig droneConfig_;        // конфігурація дрона
    AmmoType  ammo_;                 // параметри боєприпасу
    const std::unordered_map<std::string, AmmoType>* bombTypes_ = nullptr;
    double    ballisticTof_;         // time of flight (попередньо обчислений)
    double    hDistBomb_;            // horizontal distance (попередньо обчислений)
    double    maxTurnPerStep_;       // максимальний поворот за крок (попередньо обчислений)

    // State machine
    std::unique_ptr<IDroneState> currentState_;
    DroneContext ctx_;               // спільні дані для станів

    // Внутрішній стан симуляції
    std::vector<SimStep> simSteps_;  // вектор для зберігання кроків симуляції
    SimStep   simStep_ = {};         // поточний шаг симуляції
    int       currentStep_;          // лічильник кроків симуляції
    int       targetCount_;          // кількість цілей

    float     currentSpeed_;         // поточна швидкість
    double    currentTime_;
    float     acceleration_;         // прискорення для розрахунку часу розгону/гальмування
    int       remainingTurningSteps_;// Кількість кроків, що залишилися для завершення повороту
    int       timeSteps_;            // кількість часових кроків у даних цілей

    bool      hasNext_;              // Наявність обчислення наступних кроків
    bool      fireReached_ = false;  // Досягнуто точки скиду (сигнал DROP)

    IDroneStateSource* stateSource_ = nullptr; // джерело стану дрона (не володіє)

    // Опційний драйвер кроку (UART/GPIO): задає пейсинг і I/O навколо step().
    // Якщо nullptr — run() пейситься через TimeProvider (файловий/HTTP режим).
    IStepDriver* driver_ = nullptr; // не володіє

    // TimeProvider для контролю частоти ітерацій місії
    std::unique_ptr<ITimeProvider> timeProvider_;

    // Багатопоточність
    std::atomic<bool> threadReady_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_{false};

 public:
    // Конструктор — приймає solver і targets через інтерфейси
    MissionProcessor(std::unique_ptr<IBallisticSolver> s, std::unique_ptr<ITargetProvider> t) : solver_(std::move(s)), targets_(std::move(t)) {};

    // Деструктор — vector auto-cleans via RAII
    ~MissionProcessor() = default;

    // Ініціалізація: завантажити конфіг через IConfigLoader, підготувати дані
    bool init(std::unique_ptr<IConfigLoader> loader, std::unique_ptr<IResultWriter> writer, IDroneStateSource* stateSource);

    // Перевірити, чи є ще необроблені цілі
    bool hasNext();

    // Обробити наступну ціль: взяти дані з targets, обчислити через solver, повернути DropPoint
    SimStep step();

    // Тіло потоку місії
    void run();
    void start();
    void stop();
    bool isThreadReady() const;

    // Встановити драйвер кроку (UART/GPIO). Не передає володіння.
    void setStepDriver(IStepDriver* driver) { driver_ = driver; }

    // Почати ітерацію спочатку
    void reset();

    // Підмінити solver на льоту (Стратегія)
    void changeSolver(std::unique_ptr<IBallisticSolver> s) { solver_ = std::move(s); };

    // Отримати поточну позицію дрона (для зовнішнього використання)
    Coord getCurrentPos() const { return simSteps_[currentStep_].pos; }

    // Отримати поточний час
    double getCurrentTime() const { return currentTime_; }

    // Отримати кількість цілей
    int getTargetCount() const { return targets_->getTargetCount(); };

    bool exportResults();

    // ---- UART-режим: сигнал скиду і масштаби для модуля керування ----
    // true, коли місія вирішила, що дрон у точці скиду й зорієнтований на ціль.
    bool shouldDrop() const { return fireReached_; }

    // Максимальний поворот за крок місії (рад) — для нормування turnRate.
    double getMaxTurnPerStep() const { return maxTurnPerStep_; }

    // Максимальна зміна швидкості за крок місії (м/с) — для нормування accel.
    float getAccelPerStep() const { return acceleration_ * droneConfig_.simTimeStep; }

    // Отримати назву поточного стану (для логів)
    const std::string getCurrentStateName() const {
        return currentState_ ? currentState_->name() : "none";
    }

 private:
    double normalizeAngle(double angle);
    double applyLimitedTurn(const SimStep &simStep, const double &maxTurnPerStep, const double &desiredDir);
    bool leadTarget(Coord pos, const int tgtIdx, const double &currentTime,
                  const float &attackSpeed, const float &arrayTimeStep,
                  Coord &firePos, Coord &predict);
    void initDroneConstants();
    int detectBestTarget(SimStep &simStep, const double &currentTime, const float &currentSpeed,
                   const int &remainingTurningSteps, int &bestTargetId, Coord &bestPredict);
};
