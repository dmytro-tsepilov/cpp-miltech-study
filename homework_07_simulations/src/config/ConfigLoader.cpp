#include <fstream>
#include <filesystem>
#include "common/macros.h"
#include "config/ConfigLoader.h"


using json = nlohmann::json;
namespace fs = std::filesystem;

bool FileConfigLoader::load()
{
    loadConfigFromFile();
    loadAmmoTypesFromFile();

    return true;
}

DroneConfig FileConfigLoader::getConfig()
{
    return this->dConf;
}

const std::unordered_map<std::string, AmmoType>& FileConfigLoader::getAmmoParams()
{
    return ammoTypes_;
}

void FileConfigLoader::setFolderPath(const std::string &folderPath)
{
    folderPath_ = folderPath;
}

bool FileConfigLoader::loadConfigFromFile(const std::string &filename)
{
    fs::path fullPath = fs::path(folderPath_) / filename;
    std::ifstream inputFile(fullPath);
    if (!inputFile.is_open())
    {
        LOG("Error opening ammo types file: " << fullPath);
        return false;
    }

    json data = json::parse(inputFile);

    dConf.startPos = {data["drone"]["position"]["x"], data["drone"]["position"]["y"]  };
    dConf.altitude = data["drone"]["altitude"];
    dConf.initialDir = data["drone"]["initialDirection"];
    dConf.attackSpeed = data["drone"]["attackSpeed"];
    dConf.accelPath = data["drone"]["accelerationPath"];
    dConf.ammoName = data["ammo"].get<std::string>();
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
    fs::path fullPath = fs::path(folderPath_) / filename;
    std::ifstream inputFile(fullPath);
    if (!inputFile.is_open())
    {
        LOG("Error opening ammo types file: " << fullPath);
        return false;
    }

    json data = json::parse(inputFile);

    ammoTypes_.clear();
    for (const auto& item : data) {
        AmmoType ammo;
        std::string rawName = item["name"].get<std::string>();
        ammo.name = rawName;
        std::string lowerName = rawName;
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        ammo.mass = item["mass"];
        ammo.drag = item["drag"];
        ammo.lift = item["lift"];
        ammoTypes_[lowerName] = ammo;
    }

    inputFile.close();
    return true;
}
