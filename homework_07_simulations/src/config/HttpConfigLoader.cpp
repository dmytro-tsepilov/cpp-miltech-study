#include <algorithm>
#include <cctype>
#include <string>
#include <httplib.h>

#include "common/macros.h"
#include "config/ConfigLoader.h"

#define CPPHTTPLIB_OPENSSL_SUPPORT

using json = nlohmann::json;

DroneConfig HttpConfigLoader::getConfig()
{
    return this->dConf_;
}

const std::unordered_map<std::string, AmmoType>& HttpConfigLoader::getAmmoParams()
{
    return this->ammoTypes_;
}

std::string HttpConfigLoader::getBaseUrl()
{
    if (homeWork_ == "hw3") {
        return "/" + homeWork_ + "/" + basePath_;
    } else if (homeWork_ == "hw9") {
        return "/hw3/api/hw9/tests";
    } else {
        return "/";
    }
}

std::string HttpConfigLoader::getTestName(const int testNumber)
{
    std::string rawResponse;
    downloadFile(getBaseUrl(), rawResponse);

    json data = json::parse(rawResponse);

    if (testNumber < 1 || testNumber > static_cast<int>(data.size())) {
        LOG("Invalid test number: " << testNumber << " (valid range: 1-" << static_cast<int>(data.size()) << ")");
        return "";
    }

    DEBUG("Test found: " << data[testNumber - 1]["name"].get<std::string>());

    return data[testNumber - 1]["id"].get<std::string>();
}

bool HttpConfigLoader::parseDroneConfig(const std::string &rawResponse)
{
    json data = json::parse(rawResponse);

    dConf_.startPos = {data["config"]["drone"]["position"]["x"], data["config"]["drone"]["position"]["y"]};
    dConf_.altitude = data["config"]["drone"]["altitude"];
    dConf_.initialDir = data["config"]["drone"]["initialDirection"];
    dConf_.attackSpeed = data["config"]["drone"]["attackSpeed"];
    dConf_.accelPath = data["config"]["drone"]["accelerationPath"];
    dConf_.ammoName = data["config"]["ammo"].get<std::string>();
    dConf_.arrayTimeStep = data["config"]["targetArrayTimeStep"];
    dConf_.simTimeStep = data["config"]["simulation"]["timeStep"];
    dConf_.hitRadius = data["config"]["simulation"]["hitRadius"];
    dConf_.angularSpeed = data["config"]["drone"]["angularSpeed"];
    dConf_.turnThreshold = data["config"]["drone"]["turnThreshold"];

    return true;
}

bool HttpConfigLoader::parseAmmoTypes(const std::string &rawResponse)
{
    json data = json::parse(rawResponse);

    ammoTypes_.clear();
    for (const auto& item : data["ammo"]) {
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

    return true;
}

int HttpConfigLoader::downloadFile(const std::string& fullPath, std::string &rawResponse) {
    
    httplib::Client cli(apiURL_);
    auto res = cli.Get(fullPath);
    
    if (res && res->status == 200) {
        std::string data = res->body;
        DEBUG("[" << typeid(*this).name() << "] Download completed from: size=" << data.size() << " url=" << apiURL_ << fullPath);
        rawResponse = data;
    } else {
        LOG("[" << typeid(*this).name() << "] Failed to download config: " << (res ? res->status : 0) << " url=" << apiURL_ << fullPath);
    }

    return res->status;
}

bool HttpConfigLoader::load()
{
    if (testNumber_) {
        configTestName_ = getTestName(testNumber_);
    }

    if (configTestName_.empty()) {
        LOG("Config test name is empty. Cannot load drone config.");
        return false;
    }

    // Load drone configuration
    std::string configResponse;
    int configStatus = downloadFile(getBaseUrl() + "/" + configTestName_, configResponse);
    if (configStatus != 200) {
        LOG("Failed to download drone config.");
        return false;
    }
    parseDroneConfig(configResponse);

    parseAmmoTypes(configResponse);

    return true;
}
