#pragma once

constexpr double g = 9.81;

class IBallisticSolver {
public:
    virtual ~IBallisticSolver() = default;
    virtual double calculateHorizontalDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const double &time) = 0;
    virtual double calculateTimeToTarget(const float attackSpeed, const float ammoDrag, const float ammoMass, const float ammoLift, const float zd) = 0;
};

class AnalyticalSolver : public IBallisticSolver {
public:
    double calculateHorizontalDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const double &time) override;
    double calculateTimeToTarget(const float attackSpeed, const float ammoDrag, const float ammoMass, const float ammoLift, const float zd) override;
};

// ============ IBallisticSolver Factory Function ============

enum class SolverType {
    ANALYTICAL
};

inline IBallisticSolver* createBallisticSolver(SolverType type) {
    switch (type) {
        case SolverType::ANALYTICAL:
            return new AnalyticalSolver();
        default:
            return nullptr;
    }
}
