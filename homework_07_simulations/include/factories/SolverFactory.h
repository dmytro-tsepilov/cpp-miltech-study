#pragma once

#include <memory>

#include "solvers/BallisticSolver.h"
#include "interfaces/IBallisticSolver.h"
#include "common/macros.h"

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
