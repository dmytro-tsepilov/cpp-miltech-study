#include "factories/TargetProviderFactory.h"

#include "providers/TargetLoader.h"
#include "providers/UartTargetProvider.h"
#include "common/macros.h"

#include <memory>
#include <optional>
#include <string>

std::unique_ptr<ITargetProvider> createTargetProvider(SourceType type,
                                              const std::optional<std::string>& param,
                                              const std::optional<std::string>& param2) {
    switch (type) {
        case SourceType::JSON: {
            auto folderPath = param.has_value() ? param.value() : std::string("");
            auto filename = param2.has_value() ? param2.value() : std::string("targets.json");
            return std::make_unique<ThreadSafeTargetProvider>(folderPath, filename);
        }
        case SourceType::SERIAL:
            return std::make_unique<UartTargetProvider>();
        case SourceType::HTTP:
#if ENABLE_HTTP
        {
            auto homeWork = param.has_value() && !param->empty() ? param.value() : std::string("hw3");
            auto testNumber = param2.has_value() && !param2->empty() ? std::stoi(param2.value()) : 0;
            return std::make_unique<HttpTargetProvider>(homeWork, testNumber);
        }
#else
            LOG("HTTP support is disabled. Cannot create HttpTargetProvider.");
            return nullptr;
#endif
        case SourceType::TEST:
            return std::make_unique<TestTargetProvider>();
        default:
            LOG("Unknown SourceType");
            return nullptr;
    }
}
