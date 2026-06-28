#pragma once

#include <memory>
#include <optional>
#include <string>

enum class SourceType {
    JSON,
    SERIAL,
    HTTP,
    TEST
};

class ITargetProvider;

std::unique_ptr<ITargetProvider> createTargetProvider(SourceType type,
                                              const std::optional<std::string>& param = std::nullopt,
                                              const std::optional<std::string>& param2 = std::nullopt);
