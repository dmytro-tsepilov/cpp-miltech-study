#pragma once

#include <memory>

#include "result/ResultWriter.h"

enum class DestType {
    JSON,
    API,
    DATABASE
};

inline std::unique_ptr<IResultWriter> createResultWriter(DestType type, const char* param = nullptr, const char* param2 = nullptr) {
    switch (type) {
        case DestType::JSON:
            return std::make_unique<JsonResultWriter>(param ? param : "", param2 ? param2 : "simulation.json");
        case DestType::API:
            return std::make_unique<ApiResultWriter>(param ? param : "", param2 ? param2 : "");
        case DestType::DATABASE:
            return std::make_unique<DatabaseResultWriter>(param ? param : "", param2 ? param2 : "simulation_results");
        default:
            return nullptr;
    }
}
