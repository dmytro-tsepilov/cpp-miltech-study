#pragma once

struct BallisticResult {
    double t;      // час польоту
    double hDist;  // горизонтальна дистанція
};

class IBallisticSolver {
public:
    virtual ~IBallisticSolver() = default;
    virtual BallisticResult calcluateTimeAndDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const float &zd) = 0;
};
