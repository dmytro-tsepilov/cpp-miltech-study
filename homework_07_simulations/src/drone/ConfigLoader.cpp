#include <fstream>

#include "common/macros.h"

#include "drone/ConfigLoader.h"
//#include "drone/DroneConfig.h" // Потрібно включити структуру DroneConfig

using json = nlohmann::json;

bool FileConfigLoader::load()
{
    this->loadConfigFromFile();
    this->loadAmmoTypesFromFile();

    return true;
}

DroneConfig FileConfigLoader::getConfig()
{
    return this->dConf;
}

AmmoType **FileConfigLoader::getAmmoParams(int &ammoCount)
{
    ammoCount = this->ammoCount;
    return ammoTypes;
}

void FileConfigLoader::setFolderPath(const std::string folderPath)
{
    this->folderPath = folderPath;
}

bool FileConfigLoader::loadConfigFromFile(const std::string &filename)
{
    std::ifstream inputFile(this->folderPath + filename);
    if (!inputFile.is_open())
    {
        LOG("Error opening ammo types file");
        return false;
    }

    json data = json::parse(inputFile);

    dConf.startPos = {data["drone"]["position"]["x"], data["drone"]["position"]["y"]  };
    dConf.altitude = data["drone"]["altitude"];
    dConf.initialDir = data["drone"]["initialDirection"];
    dConf.attackSpeed = data["drone"]["attackSpeed"];
    dConf.accelPath = data["drone"]["accelerationPath"];
    std::strncpy(dConf.ammoName, data["ammo"].get<std::string>().c_str(), 32);
    dConf.arrayTimeStep = data["targetArrayTimeStep"];
    dConf.simTimeStep = data["simulation"]["timeStep"];
    dConf.hitRadius = data["simulation"]["hitRadius"];
    dConf.angularSpeed = data["drone"]["angularSpeed"];
    dConf.turnThreshold = data["drone"]["turnThreshold"];

    inputFile.close();
    return true;
}

/**
 * Loads ammo types from a JSON file.
 */
bool FileConfigLoader::loadAmmoTypesFromFile(const std::string &filename)
{
    std::ifstream inputFile(this->folderPath + filename);
    if (!inputFile.is_open())
    {
        LOG("Error opening ammo types file");
        return false;
    }

    json data = json::parse(inputFile);

    ammoCount = data.size();

    ammoTypes = new AmmoType*[ammoCount];
    for (int i = 0 ; i < ammoCount; i++) {
        ammoTypes[i] = new AmmoType;
        std::strncpy(ammoTypes[i]->name, data[i]["name"].get<std::string>().c_str(), 31);
        ammoTypes[i]->mass = data[i]["mass"];
        ammoTypes[i]->drag = data[i]["drag"];
        ammoTypes[i]->lift = data[i]["lift"];
    }

    inputFile.close();
    return true;
}
