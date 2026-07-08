#include "factories/ConfigLoaderFactory.h"

#include "config/FileConfigLoader.h"
#include "config/HttpConfigLoader.h"
#include "common/macros.h"

#include <memory>
#include <string>

std::unique_ptr<IConfigLoader> createConfigLoader(ConfigType type, const std::string& param, const std::string& param2) {
    switch (type) {
        case ConfigType::JSON:
            return std::make_unique<FileConfigLoader>(param, param2);
        case ConfigType::HTTP:
#if ENABLE_HTTP
        {
            auto homeWork = !param.empty() ? param : std::string("hw3");
            auto testNumber = !param2.empty() ? std::stoi(param2) : 0;
            return std::make_unique<HttpConfigLoader>(homeWork, testNumber);
        }
#else
    LOG("HTTP support is disabled. Cannot create HttpConfigLoader.");
    return nullptr;
#endif
        default:
            return nullptr;
    }
}
