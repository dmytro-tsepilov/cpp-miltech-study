#pragma once

#include <string>
#include "json.hpp"

#include "drone/DroneConfig.h"

// ============ ITargetProvider Interface ============

class ITargetProvider {
public:
    virtual ~ITargetProvider() = default;
    virtual bool load() = 0;
    virtual int getTargetCount() = 0;
    virtual int getTimeSteps() = 0;
    virtual Target *getTarget(int index) = 0;
};

// ============ JsonTargetProvider ============

class JsonTargetProvider : public ITargetProvider {
private:
    std::string fileName_;
    std::string folderPath_;
    int targetCount;
    int timeSteps;
    Target** targets = nullptr;

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

    ~JsonTargetProvider() override
    {
        for (int i = 0; i < targetCount; i++)
            delete[] targets[i];
        delete[] targets;
        targets = nullptr;
    }
};

// ============ SerialTargetProvider ============

class SerialTargetProvider : public ITargetProvider {
private:
    std::string portName;
    int targetCount;
    int timeSteps;
    Target** targets = nullptr;

public:
    SerialTargetProvider(const std::string &port);

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target *getTarget(int index) override;

    ~SerialTargetProvider() override
    {
        for (int i = 0; i < targetCount; i++)
            delete[] targets[i];
        delete[] targets;
        targets = nullptr;
    }
};

// ============ TestTargetProvider ============

class TestTargetProvider : public ITargetProvider {
private:
    int targetCount;
    int timeSteps;
    Target** targets = nullptr;

public:
    TestTargetProvider();

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target *getTarget(int index) override;

    ~TestTargetProvider() override
    {
        for (int i = 0; i < targetCount; i++)
            delete[] targets[i];
        delete[] targets;
        targets = nullptr;
    }
};

// ============ Factory Function ============

enum class SourceType {
    JSON,
    SERIAL,
    TEST
};

inline ITargetProvider* createTargetProvider(SourceType type, const char* param = nullptr, const char* param2 = nullptr) {
    switch (type) {
        case SourceType::JSON:
            return new JsonTargetProvider(param ? param : "", param2 ? param2 : "targets.json");
        case SourceType::SERIAL:
            return new SerialTargetProvider(param ? param : "/dev/ttyUSB0");
        case SourceType::TEST:
            return new TestTargetProvider();
        default:
            return nullptr;
    }
}
