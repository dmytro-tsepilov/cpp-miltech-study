#pragma once

class IBallisticSolver {
public:
    virtual ~IBallisticSolver() = default;
    virtual double calculateHorizontalDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const double &time) = 0;
    virtual double calculateTimeToTarget(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const float &zd) = 0;
};
