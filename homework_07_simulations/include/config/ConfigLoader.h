#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include "interfaces/IConfigLoader.h"
#include "DroneConfig.h"

class FileConfigLoader : public IConfigLoader {
private:
    std::string folderPath_;
    std::string fileName_;
    DroneConfig dConf;
    std::vector<AmmoType> ammoTypes_;

public:
    FileConfigLoader (const std::string &folderPath = "", const std::string &filename = "config.json") {
        fileName_ = filename;
        folderPath_ = folderPath;
    }
    bool load() override;
    DroneConfig getConfig() override;
    const std::vector<AmmoType>& getAmmoParams() override;
    void setFolderPath(const std::string &folderPath = "");
    bool loadConfigFromFile(const std::string &filename = "config.json");
    bool loadAmmoTypesFromFile(const std::string &filename = "ammo.json");

    ~FileConfigLoader() = default;
};

// ============ Factory Function ============

enum class ConfigType {
    JSON,
    SERIAL,
    TEST
};

inline std::unique_ptr<IConfigLoader> createConfigLoader(ConfigType type, const std::string& param = "", const std::string& param2 = "") {
    switch (type) {
        case ConfigType::JSON:
            return std::make_unique<FileConfigLoader>(param, param2);
        default:
            return nullptr;
    }
}
