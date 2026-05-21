#include <fstream>

#include "common/macros.h"

#include "drone/ConfigLoader.h"
//#include "drone/DroneConfig.h" // Потрібно включити структуру DroneConfig

bool FileConfigLoader::loadConfigFromFile(const std::string &filename, DroneConfig &dConf)
{
    std::ifstream inputFile(filename);
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
