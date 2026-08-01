#include "factories/ResultWriterFactory.h"

#include "result/ResultWriter.h"

#if ENABLE_HTTP
#include "result/HttpResultWriter.h"
#endif

#include <memory>
#include <optional>
#include <string>

std::unique_ptr<IResultWriter> createResultWriter(DestType type,
        const std::optional<std::string>& param,
        const std::optional<std::string>& param2,
        const std::optional<std::string>& param3) {
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
        case DestType::HTTP:
        {
#if ENABLE_HTTP
            auto studentId = param.has_value()  ? param.value() : std::string("");
            auto testId = param2.has_value()  ? param2.value() : std::string("");
            auto baseUrl = param3.has_value() ? param3.value() : std::string("http://cppmiltech.com.ua");
            return std::make_unique<HttpResultWriter>(studentId, testId, baseUrl);
#else
            (void)type;
            (void)param;
            (void)param2;
            LOG("HTTP ResultWriter requested but ENABLE_HTTP is disabled");
            return nullptr;
#endif
        }
        default:
            return nullptr;
    }
}
