#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <nlohmann/json.hpp>

#include "interfaces/ITargetProvider.h"
#include "config/DroneConfig.h"
#include "common/macros.h"

// ============ ThreadSafeTargetProvider ============
// Провайдер цілей у власному потоці. Траєкторії — приватні дані;
// назовні віддаються лише копії поточної позиції та швидкості під м'ютексом.

class ThreadSafeTargetProvider : public ITargetProvider {
private:
    std::string fileName_;
    std::string folderPath_;
    int targetCount_ = 0;
    int timeSteps_ = 0;
    float arrayTimeStep_ = 1.0f;
    float timeScale_ = 1.0f;

    std::vector<std::vector<Coord>> trajectories_;  // приватні траєкторії
    std::vector<Target> current_;                   // поточні pos + velocity

    mutable std::mutex mutex_;
    std::atomic<bool> threadReady_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_{false};

    void updateCurrent(int nodeIdx);

public:
    ThreadSafeTargetProvider(const std::string &folderPath = "", const std::string &filename = "targets.json") {
        fileName_ = filename;
        folderPath_ = folderPath;
    }

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target getTarget(int index) override;

    void setTimings(float arrayTimeStep, float timeScale) override;
    void run() override;
    void start() override;
    void stop() override;
    bool isThreadReady() override;
};

// ============ JsonTargetProvider ============

class JsonTargetProvider : public ITargetProvider {
private:
    std::string fileName_;
    std::string folderPath_;
    int targetCount;
    int timeSteps;
    std::vector<std::vector<Coord>> targets;

public:
    JsonTargetProvider(const std::string &folderPath = "", const std::string &filename = "targets.json") {
        fileName_ = filename;
        folderPath_ = folderPath;
    }

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target getTarget(int index) override;
    void setFolderPath(const std::string folderPath = "");
};

// ============ SerialTargetProvider ============

class SerialTargetProvider : public ITargetProvider {
private:
    std::string portName;
    int targetCount;
    int timeSteps;
    std::vector<std::vector<Coord>> targets;

public:
    SerialTargetProvider(const std::string &port);

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target getTarget(int index) override;
};

// ============ TestTargetProvider ============

class TestTargetProvider : public ITargetProvider {
private:
    int targetCount;
    int timeSteps;
    std::vector<std::vector<Coord>> targets;

public:
    TestTargetProvider();

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target getTarget(int index) override;
};

// ============ HttpTargetProvider ============

#if ENABLE_HTTP
class HttpTargetProvider : public ITargetProvider {
private:
    int targetCount;
    int timeSteps;
    std::vector<std::vector<Coord>> targets;
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
    Target getTarget(int index) override;
};
#endif // ENABLE_HTTP

