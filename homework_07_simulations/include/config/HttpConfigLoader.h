#pragma once

#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>

#include "interfaces/IConfigLoader.h"
#include "DroneConfig.h"

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

