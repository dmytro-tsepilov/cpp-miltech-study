#include "common/macros.h"
#include "mission/MissionProcessor.h"

bool MissionProcessor::init(IConfigLoader* loader, IResultWriter* writer)
{
    configLoader = loader;
    resultWriter = writer;
    targets->load();

    configLoader->load();
    droneConfig = configLoader->getConfig();

    //  ------- Load ammo types and config from file   -------    
    int ammoCount = 0;
    bombTypes = configLoader->getAmmoParams(ammoCount);

    // ------- Detect Ammo Type -------
    for (int i = 0; i < ammoCount; ++i)
    {
        if (!strcasecmp(droneConfig.ammoName, bombTypes[i]->name)) {
            ammo = *bombTypes[i];
            break;
        }
    }
    if (ammo.mass == 0)
    {
        LOG("Error: Unknown ammo type");
        return false;
    }

    return true;
}

// Extrapolate target position at time t + dtAhead
Coord MissionProcessor::extrapTarget(int targetId, double currentTime, double dtAhead, float dt)
{
    int idx = (int)floor(currentTime / dt) % 60;
    int next = (idx + 1) % 60;
    Target *curT = targets->getTarget(targetId);

    Coord vPos = (curT[next] - curT[idx]) / dt;

    Coord curPos = targetInterpolation(targetId, currentTime, dt);
    return curPos + vPos * dtAhead;
}

double MissionProcessor::applyLimitedTurn(const SimStep &simStep, const double &maxTurnPerStep, const double &desiredDir)
{
    double diff = desiredDir - simStep.direction;
    float direction = simStep.direction;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;

    double turn = std::clamp(diff, -maxTurnPerStep, maxTurnPerStep);
    direction += turn;

    while (direction > M_PI) direction -= 2 * M_PI;
    while (direction < -M_PI) direction += 2 * M_PI;

    return direction;
}

// Iterative fire point calculation with lead targeting
bool MissionProcessor::leadTarget(Coord pos, const int tgtIdx, const double &currentTime,
                  const float &attackSpeed, const double &hDistBomb, const double &ballisticTof, float arrayTimeStep,
                  Coord &firePos, Coord &predict)
{
    //predict = extrapTarget(tgtIdx, currentTime, ballisticTof, arrayTimeStep);
    predict = targetInterpolation(tgtIdx, currentTime + ballisticTof, arrayTimeStep);
    firePos = predict;

    // Iterative refinement (6 iterations)
    for (int iter = 0; iter < 6; iter++)
    {
        double dxT = predict.x - pos.x;
        double dyT = predict.y - pos.y;
        double distT = std::max(std::hypot(dxT, dyT), 1e-6);

        // If target is closer than bomb range, set fire point to target position
        // (attack by positioning toward target and dropping at impact point)
        if (distT <= hDistBomb)
        {
            firePos = predict;
            return true;
        }

        firePos.x = predict.x - (dxT / distT) * hDistBomb;
        firePos.y = predict.y - (dyT / distT) * hDistBomb;

        double distToFire = firePos.distanceTo(pos);
        double tImpact = distToFire / std::max(attackSpeed, 0.1f) + ballisticTof;

        //predict = extrapTarget(tgtIdx, currentTime, tImpact, arrayTimeStep);
        predict = targetInterpolation(tgtIdx, currentTime + tImpact, arrayTimeStep);
    }

    return true;
}

int MissionProcessor::calculateFlow()
{
    SimStep simStep = {};



    //  ------- Initialize target coordinates -------
    int targetCount = targets->getTargetCount();
    //int timeSteps = jsProvider.getTimeSteps();

    // Check readed data
    if (droneConfig.attackSpeed <= 0 || droneConfig.accelPath <= 0 || droneConfig.arrayTimeStep <= 0 || droneConfig.simTimeStep <= 0 || droneConfig.angularSpeed <= 0)
    {
        LOG("Error: Invalid simulation parameters");
        return 1;
    }

    DEBUG("Input data: xd=" << droneConfig.startPos.x << " yd=" << droneConfig.startPos.y << " droneConfig.altitude=" << droneConfig.altitude << " dir=" << droneConfig.initialDir
              << " speed=" << droneConfig.attackSpeed << " accelPath=" << droneConfig.accelPath << " ammo=" << droneConfig.ammoName
              << " arrayStep=" << droneConfig.arrayTimeStep << " simStep=" << droneConfig.simTimeStep);



    // Drone state variables
    simStep.pos = droneConfig.startPos;
    simStep.direction = droneConfig.initialDir;
    simStep.state = STOPPED;
    simStep.targetIdx = 0;
    simStep.dropPoint = droneConfig.startPos;
    simStep.aimPoint = droneConfig.startPos;
    simStep.predictedTarget = droneConfig.startPos;

    float currentSpeed = 0;
    int remainingTurningSteps = 0;
    double currentTime = 0;
    const double maxTurnPerStep = droneConfig.angularSpeed * droneConfig.simTimeStep;

    // Physical parameters
    float acceleration = (droneConfig.attackSpeed * droneConfig.attackSpeed) / (2.0f * droneConfig.accelPath);

    double ballisticTof = solver->calculateTimeToTarget(droneConfig.attackSpeed, ammo.drag, ammo.mass, ammo.lift, droneConfig.altitude);
    double hDistBomb = solver->calculateHorizontalDistance(droneConfig.attackSpeed, ammo.drag, ammo.mass, ammo.lift, ballisticTof);

    // Allocate dynamic array for SimStep upfront
    SimStep* steps = new SimStep[MAX_STEPS];

    // Simulation loop
    int step = 0;
    while (step < MAX_STEPS)
    {
        // Calculate aimPoint - where the bomb will fall if dropped now
        simStep.aimPoint = simStep.pos + Coord{std::cos(simStep.direction),
                                                std::sin(simStep.direction)} * (hDistBomb);

        // Store current state directly into SimStep array
        steps[step] = simStep;

        // Print progress every 100 steps
        if (step % 10 == 0)
        {
            LOG("Step " << step << ": pos=" << simStep.pos << " state=" << (int)simStep.state
                      << " target=" << simStep.targetIdx << " speed=" << currentSpeed << " dir=" << simStep.direction);
        }

        // For each target, calculate ballistics with lead targeting
        double minTotalTime = 1e9;
        int bestTargetId = simStep.targetIdx;
        // TODO: Check init coordinates
        simStep.dropPoint = simStep.pos;
        Coord bestPredict = simStep.pos;
        bool foundValidTarget = false;

        for (int tgtId = 0; tgtId < targetCount; tgtId++)
        {
            Coord firePos = {0.0, 0.0};
            Coord predict = {0.0, 0.0};

            bool hasSolution = leadTarget(simStep.pos, tgtId, currentTime,
                                            droneConfig.attackSpeed, hDistBomb, ballisticTof, droneConfig.arrayTimeStep,
                                             firePos, predict);
            if (!hasSolution)
            {
                continue;
            }
            foundValidTarget = true;

            // Distance from drone to target
            double distToFire = firePos.distanceTo(simStep.pos);

            // Total time = distance to fire point / attack speed + time of flight
            double timeToFire = distToFire / std::max(droneConfig.attackSpeed, 0.1f);
            double totalTime = timeToFire + ballisticTof;

            DEBUG("  Target " << tgtId << ": fireX=" << firePos << " predict=" << predict
                 << " distToFire=" << distToFire << " timeToFire=" << timeToFire << " ballisticTof=" << ballisticTof
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
                        timeToStop = currentSpeed / acceleration;
                        break;
                    case MOVING:
                        timeToStop = droneConfig.attackSpeed / acceleration;
                        break;
                    case TURNING:
                        timeToStop = remainingTurningSteps * droneConfig.simTimeStep;
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

        if (!foundValidTarget)
        {
            LOG("No valid forward-drop solution for any target at step " << step);
            break;
        }

        // Check if drone reached fire point
        double distToFirePoint = simStep.dropPoint.distanceTo(simStep.pos);

        // Compute horizontal bomb range for current ammo/speed

        double distToPred = bestPredict.distanceTo(simStep.pos);
        // If drone is already within bombing range, it just needs correct orientation
        bool inBombingRange = (int)round(distToPred) < (int)round(hDistBomb + droneConfig.hitRadius);
        //bool inBombingRange = distToPred < hDistBomb + droneConfig.hitRadius;

        if (inBombingRange)
        {
            // Fire when oriented toward predicted target (attack direction)
            double atkDir = atan2(bestPredict.y - simStep.pos.y, bestPredict.x - simStep.pos.x);
            double aDiff = atkDir - simStep.direction;
            while (aDiff >  M_PI) aDiff -= 2 * M_PI;
            while (aDiff < -M_PI) aDiff += 2 * M_PI;

            // Incomplete implementation when dron stay at place and waiting some time for drop bomb
            bool inBombingTime = true; //(minTotalTime - ballisticTof) < 0.2f;

            double aDiffMult = round(std::abs(aDiff * 10));
            double angStepMult = (droneConfig.angularSpeed * droneConfig.simTimeStep) * 10;
            if (aDiffMult < angStepMult && inBombingTime)
            {
                LOG(std::fixed
                          << "Reached fire point X at step " << step
                          << " dronePos=" << simStep.pos << ""
                          << " firePoint=" << simStep.dropPoint << ""
                          << " dist=" << distToFirePoint
                          << " time=" << currentTime);
                break;
            }
        }
        else if ((int)round(distToFirePoint) < droneConfig.hitRadius)
        {
            LOG(std::fixed
                      << "Reached fire point at step " << step
                      << " dronePos=" << simStep.pos << ""
                      << " firePoint=" << simStep.dropPoint << ""
                      << " dist=" << distToFirePoint
                      << " time=" << currentTime);
            break;
        }

        // Calculate desired direction:
        // - If in bombing range: orient toward predicted target (attack direction)
        // - Otherwise: navigate to fire point
        Coord desiredPos;
        if (inBombingRange) {
            desiredPos = bestPredict - simStep.pos;
        }
        else {
            desiredPos = simStep.dropPoint - simStep.pos;
        }
        double desiredDir = atan2(desiredPos.y, desiredPos.x);

        // Normalize angle difference
        double angleDiff = desiredDir - simStep.direction;
        while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
        while (angleDiff < -M_PI) angleDiff += 2 * M_PI;

        DEBUG("Step " << step << ": angleDiff=" << angleDiff << " maxTurn=" << (droneConfig.angularSpeed * droneConfig.simTimeStep) << " state=" << (int)simStep.state);

        // State machine for drone movement
        switch (simStep.state)
        {
            case DroneState::STOPPED:
                // Check if need to turn or accelerate
                if (std::abs(angleDiff) > droneConfig.turnThreshold)
                {
                    // Need to turn first
                    simStep.state = DroneState::TURNING;
                    // Use max to ensure at least 1 step, but avoid overshoot by not adding +1
                    remainingTurningSteps = (int)std::max(1, (int)std::ceil(std::abs(angleDiff) / (droneConfig.angularSpeed * droneConfig.simTimeStep)));
                }
                else
                {
                    simStep.direction = applyLimitedTurn(simStep, maxTurnPerStep, desiredDir);
                    // Can start accelerating directly
                    simStep.state = DroneState::ACCELERATING;
                }
                break;

            case DroneState::ACCELERATING:
                // Only decelerate if actually moving — prevents TURNING→ACCEL→DECEL→TURNING oscillation at zero speed
                if (std::abs(angleDiff) > droneConfig.turnThreshold * 2 && currentSpeed > droneConfig.attackSpeed * 0.1f)
                {
                    simStep.state = DECELERATING;
                }
                else
                {
                    simStep.direction = applyLimitedTurn(simStep, maxTurnPerStep, desiredDir);
                    // Accelerate to attack speed
                    currentSpeed = std::min(droneConfig.attackSpeed, currentSpeed + acceleration * droneConfig.simTimeStep);
                    if (currentSpeed >= droneConfig.attackSpeed - 0.01)
                    {
                        simStep.state = MOVING;
                    }
                }
                break;

            case DECELERATING:
                // Decelerate
                currentSpeed = std::max(0.0f, currentSpeed - acceleration * droneConfig.simTimeStep);
                if (currentSpeed < 0.01)
                {
                    currentSpeed = 0.0f;
                    // If already roughly aligned, skip turning and start accelerating
                    if (std::abs(angleDiff) > droneConfig.turnThreshold)
                    {
                        simStep.state = TURNING;
                        // Use max to ensure at least 1 step, but avoid overshoot by not adding +1
                        remainingTurningSteps = (int)std::max(1, (int)std::ceil(std::abs(angleDiff) / (droneConfig.angularSpeed * droneConfig.simTimeStep)));
                    }
                    else
                    {
                        simStep.direction = applyLimitedTurn(simStep, maxTurnPerStep, desiredDir);
                        simStep.state = ACCELERATING;
                    }
                }
                break;

            case DroneState::TURNING:
                {
                    // Rotate towards target
                    simStep.direction = applyLimitedTurn(simStep, maxTurnPerStep, desiredDir);

                    // Recalculate angle difference after rotation
                    double newAngleDiff = desiredDir - simStep.direction;
                    while (newAngleDiff > M_PI) newAngleDiff -= 2 * M_PI;
                    while (newAngleDiff < -M_PI) newAngleDiff += 2 * M_PI;

                    DEBUG("  TURNING: angleDiff=" << angleDiff << " newAngleDiff=" << newAngleDiff << " remaining=" << remainingTurningSteps);

                    remainingTurningSteps--;
                    if (remainingTurningSteps <= 0 || std::abs(newAngleDiff) < 0.01)
                    {
                        simStep.state = ACCELERATING;
                    }
                }
                break;

            case DroneState::MOVING:
                DEBUG("  MOVING: angleDiff=" << angleDiff << " threshold=" << (droneConfig.turnThreshold * 2));
                // Check if need to turn
                if (std::abs(angleDiff) > droneConfig.turnThreshold * 2)
                {
                    simStep.state = DroneState::DECELERATING;
                }
                else
                {
                    simStep.direction = applyLimitedTurn(simStep, maxTurnPerStep, desiredDir);
                    currentSpeed = droneConfig.attackSpeed;
                }
                break;
        }

        while (simStep.direction > M_PI) simStep.direction -= 2 * M_PI;
        while (simStep.direction < -M_PI) simStep.direction += 2 * M_PI;

        // Move drone in current direction
        simStep.pos.x += cos(simStep.direction) * currentSpeed * droneConfig.simTimeStep;
        simStep.pos.y += sin(simStep.direction) * currentSpeed * droneConfig.simTimeStep;

        currentTime += droneConfig.simTimeStep;
        simStep.targetIdx = bestTargetId;
        step++;
    }

    // Save to JSON format
    resultWriter->write(steps, (step + 1));

    // Free SimStep memory
    delete[] steps;
    steps = nullptr;

    return 0;
}

Coord MissionProcessor::targetInterpolation(const int8_t &targetId, const double &time, const float &arrayTimeStep)
{
    int idx = (int)floor(time / arrayTimeStep) % 60;
    int next = (idx + 1) % 60;
    double frac = (time - idx * arrayTimeStep) / arrayTimeStep;

    Target *curT = targets->getTarget(targetId);

    return {
        curT[idx].x + (curT[next].x - curT[idx].x) * frac,
        curT[idx].y + (curT[next].y - curT[idx].y) * frac
    };
}
