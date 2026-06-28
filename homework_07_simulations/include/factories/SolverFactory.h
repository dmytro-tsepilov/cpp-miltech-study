#pragma once

#include <memory>
#include <string>

enum class SolverType {
    ANALYTICAL,
    TABLE
};

class IBallisticSolver;

std::unique_ptr<IBallisticSolver> createBallisticSolver(SolverType type, const std::string& param = "");
