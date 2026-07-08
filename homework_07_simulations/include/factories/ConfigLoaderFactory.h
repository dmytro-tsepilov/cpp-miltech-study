#pragma once

#include <memory>
#include <string>

enum class ConfigType {
    JSON,
    SERIAL,
    TEST,
    HTTP
};

class IConfigLoader;

std::unique_ptr<IConfigLoader> createConfigLoader(ConfigType type, const std::string& param = "", const std::string& param2 = "");
