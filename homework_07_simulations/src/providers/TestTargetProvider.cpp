#include "providers/TargetLoader.h"

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
