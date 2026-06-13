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


// ============ HttpConfigLoader ============

#if ENABLE_HTTP
class HttpConfigLoader : public IConfigLoader {
private:
    DroneConfig dConf_;
    std::unordered_map<std::string, AmmoType> ammoTypes_;
    std::string apiURL_ = "http://cppmiltech.com.ua";
    std::string homeWork_ = "hw3";
    int testNumber_;
    std::string basePath_ = "api/tests";
    std::string configTestName_;
    std::string ammoTestName_;

    int downloadFile(const std::string& fullPath, std::string &rawResponse);
    std::string getTestName(const int testNumber);
    std::string getBaseUrl();
    bool parseDroneConfig(const std::string &rawResponse);
    bool parseAmmoTypes(const std::string &rawResponse);

public:
    HttpConfigLoader(const std::string &homeWork = "hw3", const int &testNumber = 0) {
        homeWork_ = homeWork;
        testNumber_ = testNumber;
    };

    bool load() override;
    void setApiUrl(const std::string &apiUrl) { apiURL_ = apiUrl; }
    void setTestName(const std::string &testName) { configTestName_ = testName; }
    void setAmmoTestName(const std::string &ammoTestName) { ammoTestName_ = ammoTestName; }
    DroneConfig getConfig() override;
    const std::unordered_map<std::string, AmmoType>& getAmmoParams() override;
};
#endif // ENABLE_HTTP

