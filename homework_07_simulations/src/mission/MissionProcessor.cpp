#include <vector>
#include <unordered_map>
#include <cstring>
#include "common/macros.h"
#include "config/DroneConfig.h"
#include "mission/MissionProcessor.h"

bool MissionProcessor::init(std::unique_ptr<IConfigLoader> loader, std::unique_ptr<IResultWriter> writer)
{
    configLoader_ = std::move(loader);
    resultWriter_ = std::move(writer);

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

    reset();

    return true;
}

void MissionProcessor::initDroneConstants()
{
    // Physical parameters
    maxTurnPerStep_ = droneConfig_.angularSpeed * droneConfig_.simTimeStep;

    acceleration_ = (droneConfig_.attackSpeed * droneConfig_.attackSpeed) / (2.0f * droneConfig_.accelPath);

    ballisticTof_ = solver_->calculateTimeToTarget(droneConfig_.attackSpeed, ammo_.drag, ammo_.mass, ammo_.lift, droneConfig_.altitude);
    hDistBomb_ = solver_->calculateHorizontalDistance(droneConfig_.attackSpeed, ammo_.drag, ammo_.mass, ammo_.lift, ballisticTof_);
}

// Old implementation: not used anymore, but can be reference for target interpolation
// Extrapolate target position at time t + dtAhead
Coord MissionProcessor::extrapTarget(int targetId, double currentTime, double dtAhead, float dt)
{
    int idx = (int)floor(currentTime / dt) % 60;
    int next = (idx + 1) % 60;
    Target *curT = targets_->getTarget(targetId);

    Coord vPos = (curT[next] - curT[idx]) / dt;

    Coord curPos = targetInterpolation(targetId, currentTime, dt);
    return curPos + vPos * dtAhead;
}

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
    //predict = extrapTarget(tgtIdx, currentTime, ballisticTof, arrayTimeStep);
    predict = targetInterpolation(tgtIdx, currentTime + ballisticTof_, arrayTimeStep);
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

        //predict = extrapTarget(tgtIdx, currentTime, tImpact, arrayTimeStep);
        predict = targetInterpolation(tgtIdx, currentTime + tImpact, arrayTimeStep);
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
    // Calculate aimPoint - where the bomb will fall if dropped now
    simStep_.aimPoint = simStep_.pos + Coord{std::cos(simStep_.direction),
                                            std::sin(simStep_.direction)} * (hDistBomb_);

    // Store current state directly into SimStep array
    simSteps_[currentStep_] = simStep_;

    // Print progress every 100 steps
    if (currentStep_ % 10 == 0)
    {
        LOG("Step " << currentStep_ << ": pos=" << simStep_.pos << " state=" << (int)simStep_.state
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

    DEBUG("Step " << currentStep_ << ": angleDiff=" << angleDiff << " maxTurn=" << (droneConfig_.angularSpeed * droneConfig_.simTimeStep) << " state=" << (int)simStep_.state);

    // State machine for drone movement
    switch (simStep_.state)
    {
        case DroneState::STOPPED:
            // Check if need to turn or accelerate
            if (std::abs(angleDiff) > droneConfig_.turnThreshold)
            {
                // Need to turn first
                simStep_.state = DroneState::TURNING;
                // Use max to ensure at least 1 step, but avoid overshoot by not adding +1
                remainingTurningSteps_ = (int)std::max(1, (int)std::ceil(std::abs(angleDiff) / (droneConfig_.angularSpeed * droneConfig_.simTimeStep)));
            }
            else
            {
                simStep_.direction = applyLimitedTurn(simStep_, maxTurnPerStep_, desiredDir);
                // Can start accelerating directly
                simStep_.state = DroneState::ACCELERATING;
            }
            break;

        case DroneState::ACCELERATING:
            // Only decelerate if actually moving — prevents TURNING→ACCEL→DECEL→TURNING oscillation at zero speed
            if (std::abs(angleDiff) > droneConfig_.turnThreshold * 2 && currentSpeed_ > droneConfig_.attackSpeed * 0.1f)
            {
                simStep_.state = DECELERATING;
            }
            else
            {
                simStep_.direction = applyLimitedTurn(simStep_, maxTurnPerStep_, desiredDir);
                // Accelerate to attack speed
                currentSpeed_ = std::min(droneConfig_.attackSpeed, currentSpeed_ + acceleration_ * droneConfig_.simTimeStep);
                if (currentSpeed_ >= droneConfig_.attackSpeed - 0.01)
                {
                    simStep_.state = MOVING;
                }
            }
            break;

        case DECELERATING:
            // Decelerate
            currentSpeed_ = std::max(0.0f, currentSpeed_ - acceleration_ * droneConfig_.simTimeStep);
            if (currentSpeed_ < 0.01)
            {
                currentSpeed_ = 0.0f;
                // If already roughly aligned, skip turning and start accelerating
                if (std::abs(angleDiff) > droneConfig_.turnThreshold)
                {
                    simStep_.state = TURNING;
                    // Use max to ensure at least 1 step, but avoid overshoot by not adding +1
                    remainingTurningSteps_ = (int)std::max(1, (int)std::ceil(std::abs(angleDiff) / (droneConfig_.angularSpeed * droneConfig_.simTimeStep)));
                }
                else
                {
                    simStep_.direction = applyLimitedTurn(simStep_, maxTurnPerStep_, desiredDir);
                    simStep_.state = ACCELERATING;
                }
            }
            break;

        case DroneState::TURNING:
            {
                // Rotate towards target
                simStep_.direction = applyLimitedTurn(simStep_, maxTurnPerStep_, desiredDir);

                // Recalculate angle difference after rotation
                double newAngleDiff = desiredDir - simStep_.direction;
                newAngleDiff = normalizeAngle(newAngleDiff);

                DEBUG("  TURNING: angleDiff=" << angleDiff << " newAngleDiff=" << newAngleDiff << " remaining=" << remainingTurningSteps_);

                remainingTurningSteps_--;
                if (remainingTurningSteps_ <= 0 || std::abs(newAngleDiff) < 0.01)
                {
                    simStep_.state = ACCELERATING;
                }
            }
            break;

        case DroneState::MOVING:
            DEBUG("  MOVING: angleDiff=" << angleDiff << " threshold=" << (droneConfig_.turnThreshold * 2));
            // Check if need to turn
            if (std::abs(angleDiff) > droneConfig_.turnThreshold * 2)
            {
                simStep_.state = DroneState::DECELERATING;
            }
            else
            {
                simStep_.direction = applyLimitedTurn(simStep_, maxTurnPerStep_, desiredDir);
                currentSpeed_ = droneConfig_.attackSpeed;
            }
            break;
    }

    simStep_.direction = normalizeAngle(simStep_.direction);

    // Move drone in current direction
    simStep_.pos.x += cos(simStep_.direction) * currentSpeed_ * droneConfig_.simTimeStep;
    simStep_.pos.y += sin(simStep_.direction) * currentSpeed_ * droneConfig_.simTimeStep;

    currentTime_ += droneConfig_.simTimeStep;
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

Coord MissionProcessor::targetInterpolation(const int &targetId, const double &time, const float &arrayTimeStep)
{
    int idx = (int)floor(time / arrayTimeStep) % timeSteps_;
    int next = (idx + 1) % timeSteps_;
    double frac = (time - idx * arrayTimeStep) / arrayTimeStep;

    Target *curT = targets_->getTarget(targetId);
    if (!curT) {
        LOG("targetInterpolation: getTarget(" << targetId << ") returned nullptr");
        return {0, 0};
    }

    return {
        curT[idx].x + (curT[next].x - curT[idx].x) * frac,
        curT[idx].y + (curT[next].y - curT[idx].y) * frac
    };
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
                case STOPPED:
                    timeToStop = 0;
                    break;
                case ACCELERATING:
                case DECELERATING:
                    timeToStop = currentSpeed / acceleration_;
                    break;
                case MOVING:
                    timeToStop = droneConfig_.attackSpeed / acceleration_;
                    break;
                case TURNING:
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
    simStep_.state = STOPPED;
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
}
