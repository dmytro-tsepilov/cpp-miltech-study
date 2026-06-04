#pragma once

#include <string>
#include <optional>
#include <vector>
#include <nlohmann/json.hpp>

#include "interfaces/ITargetProvider.h"
#include "drone/DroneConfig.h"
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

    std::string downloadFile(const std::string& fullPath);
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

// ============ Factory Function ============

enum class SourceType {
    JSON,
    SERIAL,
    HTTP,
    TEST
};

inline ITargetProvider* createTargetProvider(SourceType type,
                                              const std::optional<std::string>& param = std::nullopt,
                                              const std::optional<std::string>& param2 = std::nullopt) {
    switch (type) {
        case SourceType::JSON: {
            auto folderPath = param.has_value() ? param.value() : std::string("");
            auto filename = param2.has_value() ? param2.value() : std::string("targets.json");
            return new JsonTargetProvider(folderPath, filename);
        }
        case SourceType::SERIAL:
            if (!param.has_value() || param->empty()) {
                LOG("SerialTargetProvider requires a valid port name");
                return nullptr;
            }
            return new SerialTargetProvider(param.value());
        case SourceType::HTTP: {
            auto homeWork = param.has_value() && !param->empty() ? param.value() : std::string("hw3");
            auto testNumber = param2.has_value() && !param2->empty() ? std::stoi(param2.value()) : 0;
            return new HttpTargetProvider(homeWork, testNumber);
        }
        case SourceType::TEST:
            return new TestTargetProvider();
        default:
            LOG("Unknown SourceType");
            return nullptr;
        }
}
