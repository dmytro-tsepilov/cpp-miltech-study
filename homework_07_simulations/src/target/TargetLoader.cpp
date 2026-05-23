#include <fstream>
#include <string>
#include <cstring>

#include "common/macros.h"
#include "target/TargetLoader.h"

using json = nlohmann::json;

// ============ JsonTargetProvider Implementation ============

int JsonTargetProvider::getTargetCount()
{
    return this->targetCount;
}

int JsonTargetProvider::getTimeSteps()
{
    return this->timeSteps;
}

void JsonTargetProvider::setFolderPath(const std::string folderPath)
{
    this->folderPath = folderPath;
}

bool JsonTargetProvider::load()
{
    std::ifstream inputFile(folderPath + this->fileName);
    if (!inputFile.is_open())
    {
        LOG("Error opening targets file");
        return false;
    }

    json data = json::parse(inputFile);

    targetCount = (int)data["targetCount"];
    timeSteps = (int)data["timeSteps"];

    targets = new Target*[targetCount];
    for (int i = 0; i < targetCount; i++) {
        targets[i] = new Target[timeSteps];
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = data["targets"][i]["positions"][j]["x"];
            targets[i][j].y = data["targets"][i]["positions"][j]["y"];
        }
    }

    inputFile.close();
    return true;
}

Target *JsonTargetProvider::getTarget(int index) {
    return targets[index];
}

// ============ SerialTargetProvider Implementation ============

SerialTargetProvider::SerialTargetProvider(const std::string &port)
    : portName(port), targetCount(0), timeSteps(0), targets(nullptr)
{
}

bool SerialTargetProvider::load()
{
    // TODO: Implement serial port reading logic
    LOG("SerialTargetProvider::load() - not implemented yet");
    return false;
}

int SerialTargetProvider::getTargetCount()
{
    return this->targetCount;
}

int SerialTargetProvider::getTimeSteps()
{
    return this->timeSteps;
}

Target *SerialTargetProvider::getTarget(int index)
{
    if (targets && index >= 0 && index < targetCount) {
        return targets[index];
    }
    return nullptr;
}

// ============ TestTargetProvider Implementation ============

TestTargetProvider::TestTargetProvider()
    : targetCount(0), timeSteps(0), targets(nullptr)
{
}

bool TestTargetProvider::load()
{
    // Create test data for simulation
    targetCount = 1;
    timeSteps = 60;
    targets = new Target*[targetCount];
    targets[0] = new Target[timeSteps];

    // Generate circular motion pattern for testing
    for (int j = 0; j < timeSteps; j++) {
        float angle = 2.0f * M_PI * j / timeSteps;
        targets[0][j].x = 100.0 * cos(angle);
        targets[0][j].y = 100.0 * sin(angle);
    }

    return true;
}

int TestTargetProvider::getTargetCount()
{
    return this->targetCount;
}

int TestTargetProvider::getTimeSteps()
{
    return this->timeSteps;
}

Target *TestTargetProvider::getTarget(int index)
{
    if (targets && index >= 0 && index < targetCount) {
        return targets[index];
    }
    return nullptr;
}
