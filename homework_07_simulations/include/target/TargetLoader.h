#include <ostream>
#include <string>
#include "json.hpp"

#include "drone/DroneConfig.h"

class ITargetProvider {
public:
    virtual ~ITargetProvider() = default;
    virtual bool load() = 0;
    virtual int    getTargetCount() = 0;
    virtual int getTimeSteps() = 0;
    virtual Target *getTarget(int index) = 0;
};

class JsonTargetProvider : public ITargetProvider {
private:
    std::string fileName;
    std::string folderPath;
    int targetCount;
    int timeSteps;
    Target** targets = nullptr;

public:
    JsonTargetProvider(const std::string &filename = "targets.json") {
        fileName = filename;
    }

    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    bool loadTargetsFromFile();
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
