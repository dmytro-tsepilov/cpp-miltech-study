#pragma once

#include <memory>

#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"
#include "interfaces/IBallisticSolver.h"
#include "common/macros.h"

enum class SolverType {
    ANALYTICAL,
    TABLE
};

inline std::unique_ptr<IBallisticSolver> createBallisticSolver(SolverType type, const std::string& param = "") {
    switch (type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>();
        case SolverType::TABLE:
            return std::make_unique<TableSolver>(param);
        default:
            LOG("Unknown SolverType");
            return nullptr;
    }
}
