#include <fstream>

#include "common/macros.h"
#include "target/TargetLoader.h"

using json = nlohmann::json;

bool JsonTargetProvider::load()
{
    return true;
}

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

bool JsonTargetProvider::loadTargetsFromFile()
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
