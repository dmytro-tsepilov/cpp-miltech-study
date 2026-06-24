#include <vector>
#include <unordered_map>
#include <cstring>
#include <algorithm>
#include <thread>
#include <chrono>
#include "common/macros.h"
#include "config/DroneConfig.h"
#include "mission/DronePhysics.h"
#include "mission/MissionProcessor.h"

bool MissionProcessor::init(std::unique_ptr<IConfigLoader> loader, std::unique_ptr<IResultWriter> writer, DronePhysics* physics)
{
    configLoader_ = std::move(loader);
    resultWriter_ = std::move(writer);
    physics_ = physics;

    //  ------- Initialize target coordinates -------
    if (!targets_->load()) {
        LOG("Failed to load targets");
        return false;
    }
    targetCount_ = targets_->getTargetCount();
    timeSteps_ = targets_->getTimeSteps();

    //  ------- Initialize drone configuration -------
    if (!configLoader_->load()) {
        LOG("Failed to load drone config");
        return false;
    }
    droneConfig_ = configLoader_->getConfig();

    //  ------- Load ammo types and config from file   -------
    const std::unordered_map<std::string, AmmoType>& ammoTypes = configLoader_->getAmmoParams();
    bombTypes_ = &ammoTypes;

    // ------- Detect Ammo Type by name key -------
    std::string searchKey = droneConfig_.ammoName;
    std::transform(searchKey.begin(), searchKey.end(), searchKey.begin(), ::tolower);
    auto it = ammoTypes.find(searchKey);
    if (it != ammoTypes.end()) {
        ammo_ = it->second;
    }
    if (ammo_.mass == 0)
    {
        LOG("Error: Unknown ammo type");
        return false;
    }

    // Check readed data
    if (droneConfig_.attackSpeed <= 0 || droneConfig_.accelPath <= 0 || droneConfig_.arrayTimeStep <= 0 || droneConfig_.simTimeStep <= 0 || droneConfig_.angularSpeed <= 0)
    {
        LOG("Error: Invalid simulation parameters");
        return false;
    }

    DEBUG("Input data: xd=" << droneConfig_.startPos.x << " yd=" << droneConfig_.startPos.y << " droneConfig.altitude=" << droneConfig_.altitude << " dir=" << droneConfig_.initialDir
              << " speed=" << droneConfig_.attackSpeed << " accelPath=" << droneConfig_.accelPath << " ammo=" << droneConfig_.ammoName
              << " arrayStep=" << droneConfig_.arrayTimeStep << " simStep=" << droneConfig_.simTimeStep);

    initDroneConstants();

    // Налаштувати провайдер цілей (період оновлення = arrayTimeStep)
    targets_->setTimings(droneConfig_.arrayTimeStep, droneConfig_.timeScale);

    // Налаштувати фізику дрона початковим станом
    if (physics_) {
        physics_->init(droneConfig_.startPos, droneConfig_.initialDir,
                       droneConfig_.physicsTimeStep, droneConfig_.timeScale);
    }

    reset();

    return true;
}

void MissionProcessor::initDroneConstants()
{
    // Physical parameters
    maxTurnPerStep_ = droneConfig_.angularSpeed * droneConfig_.simTimeStep;

    acceleration_ = (droneConfig_.attackSpeed * droneConfig_.attackSpeed) / (2.0f * droneConfig_.accelPath);

    BallisticResult result = solver_->calcluateTimeAndDistance(droneConfig_.attackSpeed, ammo_.drag, ammo_.mass, ammo_.lift, droneConfig_.altitude);

    ballisticTof_ = result.t;
    hDistBomb_ = result.hDist;
}

// Old implementation: not used anymore, but can be reference for target interpolation
// Extrapolate target position at time t + dtAhead
// Coord MissionProcessor::extrapTarget(int targetId, double currentTime, double dtAhead, float dt)
// {
//     int idx = (int)floor(currentTime / dt) % 60;
//     int next = (idx + 1) % 60;
//     Target *curT = targets_->getTarget(targetId);

//     Coord vPos = (curT[next] - curT[idx]) / dt;

//     Coord curPos = targetInterpolation(targetId, currentTime, dt);
//     return curPos + vPos * dtAhead;
// }

double MissionProcessor::applyLimitedTurn(const SimStep &simStep, const double &maxTurnPerStep, const double &desiredDir)
{
    double diff = desiredDir - simStep.direction;
    float direction = simStep.direction;
    diff = normalizeAngle(diff);

    double turn = std::clamp(diff, -maxTurnPerStep, maxTurnPerStep);
    direction += turn;

    direction = normalizeAngle(direction);

    return direction;
}

// Iterative fire point calculation with lead targeting
bool MissionProcessor::leadTarget(Coord pos, const int tgtIdx, const double &currentTime,
                  const float &attackSpeed, const float &arrayTimeStep,
                  Coord &firePos, Coord &predict)
{
    (void)currentTime;
    (void)arrayTimeStep;

    // Знімок цілі: поточна позиція + швидкість. Прогноз майбутньої позиції
    // через квадратичну екстраполяцію pos + velocity * dt + 0.5 * acceleration * dt^2,
    // що враховує маневр (криволінійний рух) цілі.
    Target tgt = targets_->getTarget(tgtIdx);

    predict = tgt.pos + tgt.velocity * (float)ballisticTof_ + tgt.acceleration * (float)(0.5 * ballisticTof_ * ballisticTof_);
    firePos = predict;

    // Iterative refinement (6 iterations)
    for (int iter = 0; iter < 6; iter++)
    {
        double dxT = predict.x - pos.x;
        double dyT = predict.y - pos.y;
        double distT = std::max(std::hypot(dxT, dyT), 1e-6);

        // If target is closer than bomb range, set fire point to target position
        // (attack by positioning toward target and dropping at impact point)
        if (distT <= hDistBomb_)
        {
            firePos = predict;
            return true;
        }

        firePos.x = predict.x - (dxT / distT) * hDistBomb_;
        firePos.y = predict.y - (dyT / distT) * hDistBomb_;

        double distToFire = firePos.distanceTo(pos);
        double tImpact = distToFire / std::max(attackSpeed, 0.1f) + ballisticTof_;

        // Квадратичний (по прискоренню) член справедливий лише на короткому
        // горизонті; обмежуємо його балістичним часом польоту, щоб під час
        // далекого підльоту прогноз для маневреної цілі не "вибухав".
        double tAcc = std::min(tImpact, (double)ballisticTof_);
        predict = tgt.pos + tgt.velocity * (float)tImpact + tgt.acceleration * (float)(0.5 * tAcc * tAcc);
    }

    return true;
}

double MissionProcessor::normalizeAngle(double angle) {
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

SimStep MissionProcessor::step()
{
    // Request telemetry from physics: MissionProcessor no longer stores and integrates the drone's state - it only reads it.
    DroneTelemetry tel = physics_->getTelemetry();
    simStep_.pos = tel.pos;
    simStep_.direction = tel.direction;
    currentSpeed_ = tel.speed;
    currentTime_ = tel.timeSecSinceStart;
    simStep_.timeSecSinceStart = tel.timeSecSinceStart;

    // Calculate aimPoint - where the bomb will fall if dropped now
    simStep_.aimPoint = simStep_.pos + Coord{std::cos(simStep_.direction),
                                            std::sin(simStep_.direction)} * (hDistBomb_);

    // Store current state directly into SimStep array
    simSteps_[currentStep_] = simStep_;

    // Print progress every 100 steps
    if (currentStep_ % 10 == 0)
    {
        LOG("Step " << currentStep_ << ": pos=" << simStep_.pos << " state=" << getCurrentStateName()
                    << " target=" << simStep_.targetIdx << " speed=" << currentSpeed_ << " dir=" << simStep_.direction);
    }

    // For each target, calculate ballistics with lead targeting

    int bestTargetId = simStep_.targetIdx;
    // TODO: Check init coordinates
    simStep_.dropPoint = simStep_.pos;
    Coord bestPredict = simStep_.pos;

    if (!detectBestTarget(simStep_, currentTime_, currentSpeed_, remainingTurningSteps_, bestTargetId, bestPredict))
    {
        LOG("No valid forward-drop solution for any target at step " << currentStep_);
        hasNext_ = false;
        return simStep_;
    }

    // Check if drone reached fire point
    double distToFirePoint = simStep_.dropPoint.distanceTo(simStep_.pos);

    // Compute horizontal bomb range for current ammo/speed

    double distToPred = bestPredict.distanceTo(simStep_.pos);
    // If drone is already within bombing range, it just needs correct orientation
    bool inBombingRange = (int)round(distToPred) < (int)round(hDistBomb_ + droneConfig_.hitRadius);
    //bool inBombingRange = distToPred < hDistBomb_ + droneConfig_.hitRadius;

    if (inBombingRange)
    {
        // Fire when oriented toward predicted target (attack direction)
        double atkDir = atan2(bestPredict.y - simStep_.pos.y, bestPredict.x - simStep_.pos.x);
        double aDiff = atkDir - simStep_.direction;
        aDiff = normalizeAngle(aDiff);

        // Incomplete implementation when dron stay at place and waiting some time for drop bomb
        bool inBombingTime = true; //(minTotalTime - ballisticTof_) < 0.2f;

        double aDiffMult = round(std::abs(aDiff * 10));
        double angStepMult = (droneConfig_.angularSpeed * droneConfig_.simTimeStep) * 10;
        if (aDiffMult < angStepMult && inBombingTime)
        {
            LOG(std::fixed
                        << "Reached fire point X at step " << currentStep_
                        << " dronePos=" << simStep_.pos << ""
                        << " firePoint=" << simStep_.dropPoint << ""
                        << " dist=" << distToFirePoint
                        << " time=" << currentTime_);
            hasNext_ = false;
            return simStep_;
        }
    }
    else if ((int)round(distToFirePoint) < droneConfig_.hitRadius)
    {
        LOG(std::fixed
                    << "Reached fire point at step " << currentStep_
                    << " dronePos=" << simStep_.pos << ""
                    << " firePoint=" << simStep_.dropPoint << ""
                    << " dist=" << distToFirePoint
                    << " time=" << currentTime_);
        hasNext_ = false;
        return simStep_;
    }

    // Calculate desired direction:
    // - If in bombing range: orient toward predicted target (attack direction)
    // - Otherwise: navigate to fire point
    Coord desiredPos;
    if (inBombingRange) {
        desiredPos = bestPredict - simStep_.pos;
    }
    else {
        desiredPos = simStep_.dropPoint - simStep_.pos;
    }
    double desiredDir = atan2(desiredPos.y, desiredPos.x);

    // Normalize angle difference
    double angleDiff = desiredDir - simStep_.direction;
    angleDiff = normalizeAngle(angleDiff);

    DEBUG("Step " << currentStep_ << ": angleDiff=" << angleDiff << " maxTurn=" << (droneConfig_.angularSpeed * droneConfig_.simTimeStep) << " state=" << getCurrentStateName());

    // --- Update DroneContext with desired direction ---
    ctx_.desiredDir = desiredDir;
    ctx_.targetPos = desiredPos;
    ctx_.pos = simStep_.pos;
    ctx_.direction = simStep_.direction;
    ctx_.currentSpeed = currentSpeed_;
    ctx_.cfg = &droneConfig_;
    ctx_.acceleration = acceleration_;
    ctx_.maxTurnPerStep = maxTurnPerStep_;

    // Execute current state via State pattern
    auto nextState = currentState_->execute(ctx_);
    if (nextState) {
        currentState_ = std::move(nextState);
    }

    // Sync DroneContext back to simStep_
    simStep_.pos = ctx_.pos;
    simStep_.direction = ctx_.direction;
    currentSpeed_ = ctx_.currentSpeed;

    // Get state ID from current state object
    simStep_.state = currentState_->stateId();

    simStep_.direction = normalizeAngle(simStep_.direction);

    // Send command to physics: it integrates position in its own thread.
    DroneCommand cmd;
    cmd.direction = static_cast<float>(simStep_.direction);
    cmd.speed = currentSpeed_;
    cmd.state = simStep_.state;
    physics_->setCommand(cmd);

    simStep_.targetIdx = bestTargetId;
    currentStep_++;

    return simStep_;
}

bool MissionProcessor::hasNext()
{
    if (currentStep_ < MAX_STEPS && hasNext_) {
        return true;
    }

    return false;
}

bool MissionProcessor::exportResults()
{
    // Resize vector to actual step count and save
    simSteps_.resize(currentStep_ + 1);
    resultWriter_->write(simSteps_);

    return true;
}

void MissionProcessor::run()
{
    threadReady_ = true;

    while (!started_ && !stop_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    while (hasNext() && !stop_) {
        step();
        std::this_thread::sleep_for(
            std::chrono::duration<float>(droneConfig_.simTimeStep / droneConfig_.timeScale));
    }
}

void MissionProcessor::start()
{
    started_ = true;
}

void MissionProcessor::stop()
{
    stop_ = true;
}

bool MissionProcessor::isThreadReady() const
{
    return threadReady_;
}

int MissionProcessor::detectBestTarget(SimStep &simStep, const double &currentTime, const float &currentSpeed,
        const int &remainingTurningSteps, int &bestTargetId, Coord &bestPredict)
{
    double minTotalTime = 1e9;
    bool foundValidTarget = false;

    for (int tgtId = 0; tgtId < targetCount_; tgtId++)
    {
        Coord firePos = {0.0, 0.0};
        Coord predict = {0.0, 0.0};

        bool hasSolution = leadTarget(simStep.pos, tgtId, currentTime,
                                         droneConfig_.attackSpeed, droneConfig_.arrayTimeStep,
                                             firePos, predict);
        if (!hasSolution)
        {
            continue;
        }
        foundValidTarget = true;

        // Distance from drone to target
        double distToFire = firePos.distanceTo(simStep.pos);

        // Total time = distance to fire point / attack speed + time of flight
        double timeToFire = distToFire / std::max(droneConfig_.attackSpeed, 0.1f);
        double totalTime = timeToFire + ballisticTof_;

        DEBUG("  Target " << tgtId << ": fireX=" << firePos << " predict=" << predict
                << " distToFire=" << distToFire << " timeToFire=" << timeToFire << " ballisticTof=" << ballisticTof_
                << " totalTime=" << totalTime);

        // If changing target, add time to stop
        double timeToStop = 0;
        if (tgtId != simStep.targetIdx)
        {
            switch (simStep.state)
            {
                case 0: // STOPPED
                    timeToStop = 0;
                    break;
                case 1: // ACCELERATING
                case 2: // DECELERATING
                    timeToStop = currentSpeed / acceleration_;
                    break;
                case 4: // MOVING
                    timeToStop = droneConfig_.attackSpeed / acceleration_;
                    break;
                case 3: // TURNING
                    timeToStop = remainingTurningSteps * droneConfig_.simTimeStep;
                    break;
            }
        }
        totalTime += timeToStop;

        if (totalTime < minTotalTime)
        {
            minTotalTime = totalTime;
            bestTargetId = tgtId;
            simStep.dropPoint = firePos;
            simStep.predictedTarget = predict;
            bestPredict = predict;
        }
    }

    return foundValidTarget;
}

// Reset simulation state to initial position (after init(), before first step())
void MissionProcessor::reset()
{
    // Initial drone state parasmeters
    simStep_.pos = droneConfig_.startPos;
    simStep_.direction = droneConfig_.initialDir;
    simStep_.state = 0; // STOPPED
    simStep_.targetIdx = 0;
    simStep_.dropPoint = droneConfig_.startPos;
    simStep_.aimPoint = droneConfig_.startPos;
    simStep_.predictedTarget = droneConfig_.startPos;

    hasNext_ = true;
    currentStep_ = 0;
    currentTime_ = 0;
    currentSpeed_ = 0;
    remainingTurningSteps_ = 0;

    // Pre-allocate vector for SimStep
    simSteps_.resize(MAX_STEPS);

    // Initialize DroneContext
    ctx_.pos = droneConfig_.startPos;
    ctx_.direction = droneConfig_.initialDir;
    ctx_.currentSpeed = 0.0f;
    ctx_.startPos = droneConfig_.startPos;
    ctx_.cfg = &droneConfig_;
    ctx_.acceleration = acceleration_;
    ctx_.maxTurnPerStep = static_cast<float>(maxTurnPerStep_);

    // Start with StateStopped
    currentState_ = std::make_unique<StateStopped>();
}
