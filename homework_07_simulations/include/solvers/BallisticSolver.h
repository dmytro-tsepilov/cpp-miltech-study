#pragma once

#include <memory>

#include "common/macros.h"
#include "interfaces/IBallisticSolver.h"

constexpr double g = 9.81;

class AnalyticalSolver : public IBallisticSolver {
public:
    double calculateHorizontalDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const double &time) override;
    double calculateTimeToTarget(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const float &zd) override;
};

// ============ IBallisticSolver Factory Function ============

enum class SolverType {
    ANALYTICAL
};

inline std::unique_ptr<IBallisticSolver> createBallisticSolver(SolverType type) {
    switch (type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>();
        default:
            LOG("Unknown SolverType");
            return nullptr;
    }
}
