#pragma once

#include <memory>
#include <string>

#include "interfaces/IConfigLoader.h"
#include "config/ConfigLoader.h"

enum class ConfigType {
    JSON,
    SERIAL,
    TEST,
    HTTP
};

inline std::unique_ptr<IConfigLoader> createConfigLoader(ConfigType type, const std::string& param = "", const std::string& param2 = "") {
    switch (type) {
        case ConfigType::JSON:
            return std::make_unique<FileConfigLoader>(param, param2);
        case ConfigType::HTTP: {
            auto homeWork = !param.empty() ? param : std::string("hw3");
            auto testNumber = !param2.empty() ? std::stoi(param2) : 0;
            return std::make_unique<HttpConfigLoader>(homeWork, testNumber);
        }
        default:
            return nullptr;
    }
}
