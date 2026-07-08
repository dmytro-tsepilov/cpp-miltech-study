#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include "interfaces/ITargetProvider.h"
#include "config/DroneConfig.h"
#include "common/macros.h"

// ============ JsonTargetProvider ============

class JsonTargetProvider : public ITargetProvider {
private:
    std::string fileName_;
    std::string folderPath_;
    int targetCount;
    int timeSteps;
    std::vector<std::vector<Target>> targets;

public:
    JsonTargetProvider(const std::string &folderPath = "", const std::string &filename = "targets.json") {
        fileName_ = filename;
        folderPath_ = folderPath;
    }

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target *getTarget(int index) override;
    void setFolderPath(const std::string folderPath = "");
};

// ============ SerialTargetProvider ============

class SerialTargetProvider : public ITargetProvider {
private:
    std::string portName;
    int targetCount;
    int timeSteps;
    std::vector<std::vector<Target>> targets;

public:
    SerialTargetProvider(const std::string &port);

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target *getTarget(int index) override;
};

// ============ TestTargetProvider ============

class TestTargetProvider : public ITargetProvider {
private:
    int targetCount;
    int timeSteps;
    std::vector<std::vector<Target>> targets;

public:
    TestTargetProvider();

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target *getTarget(int index) override;
};

// ============ HttpTargetProvider ============

#if ENABLE_HTTP
class HttpTargetProvider : public ITargetProvider {
private:
    int targetCount;
    int timeSteps;
    std::vector<std::vector<Target>> targets;
    std::string apiURL_ = "http://cppmiltech.com.ua";
    std::string testName_;
    int testNumber_;
    std::string basePath_ = "api/tests";
    std::string homeWork_ = "hw3";

    int downloadFile(const std::string& fullPath, std::string &rawResponse);
    std::string getTestName(const int testNumber);
    bool parseTargets(const std::string &rawResponse);

public:
    HttpTargetProvider(const std::string &homeWork = "hw3", const int &testNumber = 0) {
        homeWork_ = homeWork;
        testNumber_ = testNumber;
    };

    bool load() override;
    void setApiUrl(const std::string &apiUrl) {  apiURL_ = apiUrl; };
    std::string getBaseUrl();
    void setTestName(const std::string &testName) { testName_ = testName; }
    void setHomeWork(const std::string &homeWork) { homeWork_ = homeWork; }
    int getTargetCount() override;
    int getTimeSteps() override;
    Target *getTarget(int index) override;
};
#endif // ENABLE_HTTP

