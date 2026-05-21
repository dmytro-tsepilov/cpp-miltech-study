#include <string>
#include "json.hpp"
#include "DroneConfig.h"

using json = nlohmann::json;

// Інтерфейс
class IConfigLoader {
public:
    virtual ~IConfigLoader() = default;
    virtual bool loadConfigFromFile(const std::string &filename, DroneConfig &dConf) = 0;
};

// Реалізація для файлів
class FileConfigLoader : public IConfigLoader {
public:
    bool loadConfigFromFile(const std::string &filename, DroneConfig &dConf) override;
};
