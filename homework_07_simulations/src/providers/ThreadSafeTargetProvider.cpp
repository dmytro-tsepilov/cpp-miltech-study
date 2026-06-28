#include <fstream>
#include <filesystem>
#include <thread>
#include <chrono>

#include "common/macros.h"
#include "providers/TargetLoader.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

// ============ ThreadSafeTargetProvider Implementation ============

int ThreadSafeTargetProvider::getTargetCount()
{
    return this->targetCount_;
}

int ThreadSafeTargetProvider::getTimeSteps()
{
    return this->timeSteps_;
}

bool ThreadSafeTargetProvider::load()
{
    fs::path fullPath = fs::path(folderPath_) / fileName_;
    std::ifstream inputFile(fullPath);
    if (!inputFile.is_open())
    {
        LOG("Error opening targets file: " << fullPath);
        return false;
    }

    json data = json::parse(inputFile);

    targetCount_ = (int)data["targetCount"];
    timeSteps_ = (int)data["timeSteps"];

    trajectories_.assign(targetCount_, std::vector<Coord>(timeSteps_));
    for (int i = 0; i < targetCount_; i++) {
        for (int j = 0; j < timeSteps_; j++) {
            trajectories_[i][j].x = data["targets"][i]["positions"][j]["x"];
            trajectories_[i][j].y = data["targets"][i]["positions"][j]["y"];
        }
    }

    current_.assign(targetCount_, Target{});

    // Початкові значення (нульова швидкість, поки невідомий arrayTimeStep)
    for (int i = 0; i < targetCount_; i++) {
        current_[i].pos = trajectories_[i].empty() ? Coord{0, 0} : trajectories_[i][0];
        current_[i].velocity = {0, 0};
        current_[i].acceleration = {0, 0};
    }

    inputFile.close();
    return true;
}

void ThreadSafeTargetProvider::updateCurrent(int nodeIdx)
{
    if (timeSteps_ <= 0) {
        return;
    }

    int idx = ((nodeIdx % timeSteps_) + timeSteps_) % timeSteps_;
    int next = (idx + 1) % timeSteps_;
    int prev = ((idx - 1) % timeSteps_ + timeSteps_) % timeSteps_;

    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < targetCount_; i++) {
        Coord pos = trajectories_[i][idx];
        Coord velocity = (trajectories_[i][next] - trajectories_[i][idx]) / arrayTimeStep_;

        // Target acceleration (second central difference) — allows quadratic
        // prediction for maneuvering/curved targets instead of linear.
        Coord acceleration = (trajectories_[i][next] - trajectories_[i][idx] * 2.0f + trajectories_[i][prev]) / (arrayTimeStep_ * arrayTimeStep_);
        current_[i].pos = pos;
        current_[i].velocity = velocity;
        current_[i].acceleration = acceleration;
    }
}

Target ThreadSafeTargetProvider::getTarget(int index)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < 0 || index >= targetCount_) {
        LOG("ThreadSafeTargetProvider::getTarget() - index " << index << " out of range [0, " << targetCount_ << ")");
        return Target{};
    }
    return current_[index];
}

void ThreadSafeTargetProvider::setTimings(float arrayTimeStep, float timeScale)
{
    arrayTimeStep_ = arrayTimeStep > 0.0f ? arrayTimeStep : 1.0f;
    timeScale_ = timeScale > 0.0f ? timeScale : 1.0f;
    updateCurrent(0);
}

void ThreadSafeTargetProvider::run()
{
    threadReady_ = true;

    while (!started_ && !stop_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    int nodeIdx = 0;
    while (!stop_) {
        std::this_thread::sleep_for(std::chrono::duration<float>(arrayTimeStep_ / timeScale_));
        if (stop_) {
            break;
        }
        nodeIdx = (nodeIdx + 1) % (timeSteps_ > 0 ? timeSteps_ : 1);
        updateCurrent(nodeIdx);
    }
}

void ThreadSafeTargetProvider::start()
{
    started_ = true;
}

void ThreadSafeTargetProvider::stop()
{
    stop_ = true;
}

bool ThreadSafeTargetProvider::isThreadReady()
{
    return threadReady_;
}
