#include "common/macros.h"
#include "providers/TargetLoader.h"

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
