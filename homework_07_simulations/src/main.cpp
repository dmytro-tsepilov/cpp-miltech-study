#include <iostream>
#include <fstream>
#include <cmath>
#include <strings.h>

#include "common/macros.h"

// Вимикаємо попередження про тавтологічні порівняння перед інклудом json.hpp
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtautological-overlap-compare"

#include "json.hpp"
// Повертаємо попередження назад для решти коду
#pragma GCC diagnostic pop

using json = nlohmann::json;

#include "drone/ConfigLoader.h"

std::string DATA_FOLDER = "./";

constexpr double g = 9.81;

enum DroneState : int8_t
{
    STOPPED      = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING      = 3,
    MOVING       = 4,
};

struct SimStep {
	Coord pos;          	// позиція дрона
	float direction;    	// напрямок (рад)
	int8_t state;        	// стан автомата (0-4)
	int   targetIdx;    	// індекс поточної цілі
	Coord dropPoint;    	// точка скиду (куди летить дрон)
	Coord aimPoint;     	// куди впаде бомба (якщо скинути зараз)
	Coord predictedTarget;  // прогнозована позиція цілі
};

const int MAX_STEPS = 10000;

// Traget coordintes arrays
Coord** targets = nullptr;


bool loadTargetsFromFile(int &targetCount, int &timeSteps, Coord **&targets, const std::string &filename = "targets.json");
double calculateTimeToTarget(const float attackSpeed, const float ammoDrag, const float ammoMass, const float ammoLift, const float zd);
double calculateHorizontalDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const double &time);
Coord targetInterpolation(const int8_t &targetId, const double &time, const float &arrayTimeStep);
Coord extrapTarget(int targetId, double currentTime, double dtAhead, float dt);
double applyLimitedTurn(const SimStep &simStep, const double &maxTurnPerStep, const double &desiredDir);
bool leadTarget(Coord pos, float zd, int tgtIdx, const double &currentTime, const float &attackSpeed,
                  const AmmoType &ammoType, float arrayTimeStep, Coord &firePos, Coord &predict);

int calculateFlow();
bool saveResultsToJson(SimStep* steps, int stepCount, const std::string &filename = "simulation.json");

int main(int argc, char** argv)
{
    // The executable expects folder path with simulation files
    if (argc != 2) {
        std::cerr << "usage: drone_simulations <input_path>\n";
        return 1;
    }

    DATA_FOLDER = argv[1];

    calculateFlow();

    return 0;
}

// Extrapolate target position at time t + dtAhead
Coord extrapTarget(int targetId, double currentTime, double dtAhead, float dt)
{
    int idx = (int)floor(currentTime / dt) % 60;
    int next = (idx + 1) % 60;
    Coord vPos = (targets[targetId][next] - targets[targetId][idx]) / dt;

    Coord curPos = targetInterpolation(targetId, currentTime, dt);
    return curPos + vPos * dtAhead;
}

double applyLimitedTurn(const SimStep &simStep, const double &maxTurnPerStep, const double &desiredDir)
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
bool leadTarget(Coord pos, float zd, int tgtIdx, const double &currentTime,
                  const float &attackSpeed, const AmmoType &ammoType, float arrayTimeStep,
                  Coord &firePos, Coord &predict)
{
    double tof = calculateTimeToTarget(attackSpeed, ammoType.drag, ammoType.mass, ammoType.lift, zd);
    double hDist = calculateHorizontalDistance(attackSpeed, ammoType.drag, ammoType.mass, ammoType.lift, tof);

    //predict = extrapTarget(tgtIdx, currentTime, tof, arrayTimeStep);
    predict = targetInterpolation(tgtIdx, currentTime + tof, arrayTimeStep);
    firePos = predict;

    // Iterative refinement (6 iterations)
    for (int iter = 0; iter < 6; iter++)
    {
        double dxT = predict.x - pos.x;
        double dyT = predict.y - pos.y;
        double distT = std::max(std::hypot(dxT, dyT), 1e-6);

        // If target is closer than bomb range, set fire point to target position
        // (attack by positioning toward target and dropping at impact point)
        if (distT <= hDist)
        {
            firePos = predict;
            return true;
        }

        firePos.x = predict.x - (dxT / distT) * hDist;
        firePos.y = predict.y - (dyT / distT) * hDist;

        double distToFire = firePos.distanceTo(pos);
        double tImpact = distToFire / std::max(attackSpeed, 0.1f) + tof;

        //predict = extrapTarget(tgtIdx, currentTime, tImpact, arrayTimeStep);
        predict = targetInterpolation(tgtIdx, currentTime + tImpact, arrayTimeStep);
    }

    return true;
}

int calculateFlow()
{
    DroneConfig dConf;
    AmmoType ammo = {};
    SimStep simStep = {};

    //  ------- Load ammo types and config from file   -------
    AmmoType** bombTypes = nullptr;
    int ammoCount = 0;

    FileConfigLoader *cfgLoader = new FileConfigLoader();
    cfgLoader->setFolderPath(DATA_FOLDER);
    cfgLoader->load();
    dConf = cfgLoader->getConfig();
    bombTypes = cfgLoader->getAmmoParams(ammoCount);

    //  ------- Initialize target coordinates -------
    int targetCount = 0;
    int timeSteps = 0;

    loadTargetsFromFile(targetCount, timeSteps, targets);

    // Check readed data

    if (dConf.attackSpeed <= 0 || dConf.accelPath <= 0 || dConf.arrayTimeStep <= 0 || dConf.simTimeStep <= 0 || dConf.angularSpeed <= 0)
    {
        LOG("Error: Invalid simulation parameters");
        return 1;
    }

    DEBUG("Input data: xd=" << dConf.startPos.x << " yd=" << dConf.startPos.y << " dConf.altitude=" << dConf.altitude << " dir=" << dConf.initialDir
              << " speed=" << dConf.attackSpeed << " accelPath=" << dConf.accelPath << " ammo=" << dConf.ammoName
              << " arrayStep=" << dConf.arrayTimeStep << " simStep=" << dConf.simTimeStep);

    // ------- Detect Ammo Type -------
    for (int i = 0; i < ammoCount; ++i)
    {
        if (!strcasecmp(dConf.ammoName, bombTypes[i]->name)) {
            ammo = *bombTypes[i];
            break;
        }
    }
    if (ammo.mass == 0)
    {
        LOG("Error: Unknown ammo type");
        return 1;
    }

    // Drone state variables
    simStep.pos = dConf.startPos;
    simStep.direction = dConf.initialDir;
    simStep.state = STOPPED;
    simStep.targetIdx = 0;
    simStep.dropPoint = dConf.startPos;
    simStep.aimPoint = dConf.startPos;
    simStep.predictedTarget = dConf.startPos;

    float currentSpeed = 0;
    int remainingTurningSteps = 0;
    double currentTime = 0;
    const double maxTurnPerStep = dConf.angularSpeed * dConf.simTimeStep;

    // Physical parameters
    float acceleration = (dConf.attackSpeed * dConf.attackSpeed) / (2.0f * dConf.accelPath);
    double ballisticTof = calculateTimeToTarget(dConf.attackSpeed, ammo.drag, ammo.mass, ammo.lift, dConf.altitude);
    double hDistBomb = calculateHorizontalDistance(dConf.attackSpeed, ammo.drag, ammo.mass, ammo.lift, ballisticTof);

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

            bool hasSolution = leadTarget(simStep.pos, dConf.altitude, tgtId, currentTime,
                                            dConf.attackSpeed, ammo, dConf.arrayTimeStep,
                                            firePos, predict);
            if (!hasSolution)
            {
                continue;
            }
            foundValidTarget = true;

            // Distance from drone to target
            double distToFire = firePos.distanceTo(simStep.pos);

            // Total time = distance to fire point / attack speed + time of flight
            double timeToFire = distToFire / std::max(dConf.attackSpeed, 0.1f);
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
                        timeToStop = dConf.attackSpeed / acceleration;
                        break;
                    case TURNING:
                        timeToStop = remainingTurningSteps * dConf.simTimeStep;
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
        bool inBombingRange = (int)round(distToPred) < (int)round(hDistBomb + dConf.hitRadius);
        //bool inBombingRange = distToPred < hDistBomb + dConf.hitRadius;

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
            double angStepMult = (dConf.angularSpeed * dConf.simTimeStep) * 10;
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
        else if ((int)round(distToFirePoint) < dConf.hitRadius)
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

        DEBUG("Step " << step << ": angleDiff=" << angleDiff << " maxTurn=" << (dConf.angularSpeed * dConf.simTimeStep) << " state=" << (int)simStep.state);

        // State machine for drone movement
        switch (simStep.state)
        {
            case DroneState::STOPPED:
                // Check if need to turn or accelerate
                if (std::abs(angleDiff) > dConf.turnThreshold)
                {
                    // Need to turn first
                    simStep.state = DroneState::TURNING;
                    // Use max to ensure at least 1 step, but avoid overshoot by not adding +1
                    remainingTurningSteps = (int)std::max(1, (int)std::ceil(std::abs(angleDiff) / (dConf.angularSpeed * dConf.simTimeStep)));
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
                if (std::abs(angleDiff) > dConf.turnThreshold * 2 && currentSpeed > dConf.attackSpeed * 0.1f)
                {
                    simStep.state = DECELERATING;
                }
                else
                {
                    simStep.direction = applyLimitedTurn(simStep, maxTurnPerStep, desiredDir);
                    // Accelerate to attack speed
                    currentSpeed = std::min(dConf.attackSpeed, currentSpeed + acceleration * dConf.simTimeStep);
                    if (currentSpeed >= dConf.attackSpeed - 0.01)
                    {
                        simStep.state = MOVING;
                    }
                }
                break;

            case DECELERATING:
                // Decelerate
                currentSpeed = std::max(0.0f, currentSpeed - acceleration * dConf.simTimeStep);
                if (currentSpeed < 0.01)
                {
                    currentSpeed = 0.0f;
                    // If already roughly aligned, skip turning and start accelerating
                    if (std::abs(angleDiff) > dConf.turnThreshold)
                    {
                        simStep.state = TURNING;
                        // Use max to ensure at least 1 step, but avoid overshoot by not adding +1
                        remainingTurningSteps = (int)std::max(1, (int)std::ceil(std::abs(angleDiff) / (dConf.angularSpeed * dConf.simTimeStep)));
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
                DEBUG("  MOVING: angleDiff=" << angleDiff << " threshold=" << (dConf.turnThreshold * 2));
                // Check if need to turn
                if (std::abs(angleDiff) > dConf.turnThreshold * 2)
                {
                    simStep.state = DroneState::DECELERATING;
                }
                else
                {
                    simStep.direction = applyLimitedTurn(simStep, maxTurnPerStep, desiredDir);
                    currentSpeed = dConf.attackSpeed;
                }
                break;
        }

        while (simStep.direction > M_PI) simStep.direction -= 2 * M_PI;
        while (simStep.direction < -M_PI) simStep.direction += 2 * M_PI;

        // Move drone in current direction
        simStep.pos.x += cos(simStep.direction) * currentSpeed * dConf.simTimeStep;
        simStep.pos.y += sin(simStep.direction) * currentSpeed * dConf.simTimeStep;

        currentTime += dConf.simTimeStep;
        simStep.targetIdx = bestTargetId;
        step++;
    }

    // Save to JSON format
    saveResultsToJson(steps, (step + 1));

    // Free SimStep memory
    delete[] steps;
    steps = nullptr;

    for (int i = 0; i < ammoCount; i++)
        delete bombTypes[i];
    delete[] bombTypes;
    bombTypes = nullptr;

    for (int i = 0; i < targetCount; i++)
        delete[] targets[i];
    delete[] targets;
    targets = nullptr;

    return 0;
}

double calculateTimeToTarget(const float attackSpeed, const float ammoDrag, const float ammoMass, const float ammoLift, const float zd)
{
    // Calculate time to target
    double a = ammoDrag * g * ammoMass - 2 * pow(ammoDrag, 2) * ammoLift * attackSpeed;
    double b = -3 * g * pow(ammoMass, 2) + 3 * ammoDrag * ammoLift * ammoMass * attackSpeed;
    double c = 6 * pow(ammoMass, 2) * zd;

    // Degenerate case: a ≈ 0 → use simple free fall formula
    if (std::abs(a) < 1e-12)
    {
        return std::sqrt(2 * zd / g);
    }

    // Calculate Kardano method
    double p = -pow(b, 2) / (3 * pow(a, 2));
    double q = (2 * pow(b, 3)) / (27 * pow(a, 3)) + c / a;

    // If p >= 0, use fallback formula
    if (p >= 0)
    {
        return std::sqrt(2 * zd / g);
    }

    double arg = 3 * q / (2 * p) * std::sqrt(-3 / p);

    // If arg outside [-1, 1], use fallback formula
    if (std::abs(arg) > 1)
    {
        return std::sqrt(2 * zd / g);
    }

    double phi = std::acos(arg);
    double t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * M_PI) / 3) - b / (3 * a);

    // If computed t is invalid, use fallback
    if (t <= 0 || !std::isfinite(t))
    {
        return std::sqrt(2 * zd / g);
    }

    return t;
}

double calculateHorizontalDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const double &time)
{
    // Calclulate horizontal distance to target
    // h = V₀t − t²d·V₀/(2m) + t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
    //     + t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³)
    //     + t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)

    // h = V₀t − t²d·V₀/(2m) +
    double horizontalDistance = attackSpeed * time - pow(time, 2) * ammoDrag * attackSpeed / (2 * ammoMass) +
                               // t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
                               pow(time, 3) * (6 * ammoDrag * g * ammoLift * ammoMass - 6 * pow(ammoDrag, 2) * (pow(ammoLift, 2) - 1) * attackSpeed) / (36 * pow(ammoMass, 2)) +
                               // t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³) +
                               pow(time, 4) * (-6 * pow(ammoDrag, 2) * g * ammoLift * (1 + pow(ammoLift, 2) + pow(ammoLift, 4)) * ammoMass + 3 * pow(ammoDrag, 3) * pow(ammoLift, 2) * (1 + pow(ammoLift, 2)) * attackSpeed + 6 * pow(ammoDrag, 3) * pow(ammoLift, 4) * (1 + pow(ammoLift, 2)) * attackSpeed) / (36 * pow(1 + pow(ammoLift, 2), 2) * pow(ammoMass, 3)) +
                               // t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
                               pow(time, 5) * (3 * pow(ammoDrag, 3) * g * pow(ammoLift, 3) * ammoMass - 3 * pow(ammoDrag, 4) * pow(ammoLift, 2) * (1 + pow(ammoLift, 2)) * attackSpeed) / (36 * (1 + pow(ammoLift, 2)) * pow(ammoMass, 4));

    return horizontalDistance;
}

Coord targetInterpolation(const int8_t &targetId, const double &time, const float &arrayTimeStep)
{
    int idx = (int)floor(time / arrayTimeStep) % 60;
    int next = (idx + 1) % 60;
    double frac = (time - idx * arrayTimeStep) / arrayTimeStep;
    return {
        targets[targetId][idx].x + (targets[targetId][next].x - targets[targetId][idx].x) * frac,
        targets[targetId][idx].y + (targets[targetId][next].y - targets[targetId][idx].y) * frac
    };
}

bool loadTargetsFromFile(int &targetCount, int &timeSteps, Coord **&targets, const std::string &filename)
{
    std::ifstream inputFile(DATA_FOLDER + filename);
    if (!inputFile.is_open())
    {
        LOG("Error opening targets file");
        return false;
    }

    json data = json::parse(inputFile);

    targetCount = (int)data["targetCount"];
    timeSteps = (int)data["timeSteps"];

    targets = new Coord*[targetCount];
    for (int i = 0; i < targetCount; i++) {
        targets[i] = new Coord[timeSteps];
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = data["targets"][i]["positions"][j]["x"];
            targets[i][j].y = data["targets"][i]["positions"][j]["y"];
        }
    }

    inputFile.close();
    return true;
}

bool saveResultsToJson(SimStep* steps, int stepCount, const std::string &filename)
{
    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        LOG("Error opening " << filename << " for writing");
        return false;
    }

    LOG("Writing " << stepCount << " steps to " << filename << " (JSON format)");

    json out;
    out["totalSteps"] = stepCount;
    out["steps"] = json::array();

    for (int i = 0; i < stepCount; i++)
    {
        json step;
        step["position"] = {{"x", steps[i].pos.x}, {"y", steps[i].pos.y}};
        step["direction"] = steps[i].direction;
        step["state"] = steps[i].state;
        step["targetIndex"] = steps[i].targetIdx;
        step["dropPoint"] = {{"x", steps[i].dropPoint.x}, {"y", steps[i].dropPoint.y}};
        step["aimPoint"] = {{"x", steps[i].aimPoint.x}, {"y", steps[i].aimPoint.y}};
        step["predictedTarget"] = {{"x", steps[i].predictedTarget.x}, {"y", steps[i].predictedTarget.y}};
        out["steps"].push_back(step);
    }

    outFile << out.dump(2);  // 2 = indent for readability
    outFile.close();

    LOG("JSON simulation completed: " << stepCount << " steps written to " << filename);

    return true;
}


class ITargetProvider {
public:
    virtual int    getTargetCount() = 0;
    virtual Target getTarget(int index) = 0;
    virtual ~ITargetProvider() {}
};

class JsonTargetProvider : public ITargetProvider {
public:
    JsonTargetProvider(const std::string &filename) {
        // Load targets from JSON file
    }

    int getTargetCount() override {
        // Return number of targets
        return 0;
    }

    Target getTarget(int index) override {
        // Return target at specified index
        return Target();
    }
};
