#pragma once

#include <string>
#include <nlohmann/json.hpp>

#include "interfaces/IConfigLoader.h"
#include "DroneConfig.h"

class FileConfigLoader : public IConfigLoader {
private:
    std::string folderPath_;
    std::string fileName_;
    DroneConfig dConf;
    AmmoType** ammoTypes_ = nullptr;
    int ammoCount_;

public:
    FileConfigLoader (const std::string &folderPath = "", const std::string &filename = "config.json") {
        fileName_ = filename;
        folderPath_ = folderPath;
    }
    bool load() override;
    DroneConfig getConfig() override;
    AmmoType **getAmmoParams(int &ammoCount) override;
    void setFolderPath(const std::string &folderPath = "");
    bool loadConfigFromFile(const std::string &filename = "config.json");
    bool loadAmmoTypesFromFile(const std::string &filename = "ammo.json");

    ~FileConfigLoader() {
        for (int i = 0; i < ammoCount_; i++)
            delete ammoTypes_[i];
        delete[] ammoTypes_;
        ammoTypes_ = nullptr; 
    }
};

// ============ Factory Function ============

enum class ConfigType {
    JSON,
    SERIAL,
    TEST
};

inline IConfigLoader* createConfigLoader(ConfigType type, const std::string& param = "", const std::string& param2 = "") {
    switch (type) {
        case ConfigType::JSON:
            return new FileConfigLoader(param, param2);
        default:
            return nullptr;
    }
}
