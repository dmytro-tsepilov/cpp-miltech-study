#pragma once

#include <memory>
#include <optional>
#include <string>

enum class DestType {
    JSON,
    API,
    DATABASE,
    HTTP,
    SOCKET
};

class IResultWriter;

std::unique_ptr<IResultWriter> createResultWriter(DestType type,
        const std::optional<std::string>& param = std::nullopt,
        const std::optional<std::string>& param2 = std::nullopt,
        const std::optional<std::string>& param3 = std::nullopt
    );
