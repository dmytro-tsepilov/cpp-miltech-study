#include "factories/SolverFactory.h"

#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"
#include "common/macros.h"

#include <memory>
#include <string>

std::unique_ptr<IBallisticSolver> createBallisticSolver(SolverType type, const std::string& param) {
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
