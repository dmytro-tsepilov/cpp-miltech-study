#pragma once

#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>

#include "interfaces/IConfigLoader.h"
#include "DroneConfig.h"

class FileConfigLoader : public IConfigLoader {
private:
    std::string folderPath_;
    std::string fileName_;
    DroneConfig dConf;
    std::unordered_map<std::string, AmmoType> ammoTypes_;

public:
    FileConfigLoader (const std::string &folderPath = "", const std::string &filename = "config.json") {
        fileName_ = filename;
        folderPath_ = folderPath;
    }
    bool load() override;
    DroneConfig getConfig() override;
    const std::unordered_map<std::string, AmmoType>& getAmmoParams() override;
    void setFolderPath(const std::string &folderPath = "");
    bool loadConfigFromFile(const std::string &filename = "config.json");
    bool loadAmmoTypesFromFile(const std::string &filename = "ammo.json");

    ~FileConfigLoader() = default;
};
