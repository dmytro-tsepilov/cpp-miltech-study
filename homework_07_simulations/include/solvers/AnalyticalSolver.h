#pragma once

#include "interfaces/IBallisticSolver.h"

constexpr double g = 9.81;

class AnalyticalSolver : public IBallisticSolver {
public:
    BallisticResult calcluateTimeAndDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const float &zd) override;
    double calculateHorizontalDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const double &time);
    double calculateTimeToTarget(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const float &zd);
};
