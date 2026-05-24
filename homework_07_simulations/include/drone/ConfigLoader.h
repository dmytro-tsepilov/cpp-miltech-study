#pragma once

#include <string>
#include "json.hpp"

#include "DroneConfig.h"

// Інтерфейс
class IConfigLoader {
public:
    virtual ~IConfigLoader() = default;
    virtual bool load() = 0;

    virtual DroneConfig getConfig() = 0;
    virtual AmmoType **getAmmoParams(int &ammoCount) = 0;
};

class FileConfigLoader : public IConfigLoader {
private:
    std::string folderPath;
    DroneConfig dConf;
    AmmoType** ammoTypes = nullptr;
    int ammoCount;

public:
    bool load() override;
    DroneConfig getConfig() override;
    AmmoType **getAmmoParams(int &ammoCount) override;
    void setFolderPath(const std::string folderPath = "");
    bool loadConfigFromFile(const std::string &filename = "config.json");
    bool loadAmmoTypesFromFile(const std::string &filename = "ammo.json");

    ~FileConfigLoader() {
        for (int i = 0; i < ammoCount; i++)
            delete ammoTypes[i];
        delete[] ammoTypes;
        ammoTypes = nullptr; 
    }
};
