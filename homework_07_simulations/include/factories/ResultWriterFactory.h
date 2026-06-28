#pragma once

#include <memory>

#include "result/ResultWriter.h"

enum class DestType {
    JSON,
    API,
    DATABASE
};

inline std::unique_ptr<IResultWriter> createResultWriter(DestType type,
        const std::optional<std::string>& param = std::nullopt,
        const std::optional<std::string>& param2 = std::nullopt) {
    switch (type) {
        case DestType::JSON:
        {
            auto folderPath = param.has_value() ? param.value() : std::string("");
            auto filename = param2.has_value() ? param2.value() : std::string("simulation.json");
            return std::make_unique<JsonResultWriter>(folderPath, filename);
        }
        case DestType::API:
        {
            auto apiUrl = param.has_value() ? param.value() : std::string("");
            auto authToken = param2.has_value() ? param2.value() : std::string("");
            return std::make_unique<ApiResultWriter>(apiUrl, authToken);
        }
        case DestType::DATABASE:
        {
            auto connectionString = param.has_value() ? param.value() : std::string("");
            auto tableName = param2.has_value() ? param2.value() : std::string("");
            return std::make_unique<DatabaseResultWriter>(connectionString, tableName);
        }
        default:
            return nullptr;
    }
}
