#include <string>
#include <httplib.h>

#include "common/macros.h"
#include "providers/TargetLoader.h"

#define CPPHTTPLIB_OPENSSL_SUPPORT

using json = nlohmann::json;

int HttpTargetProvider::getTargetCount()
{
    return this->targetCount;
}

int HttpTargetProvider::getTimeSteps()
{
    return this->timeSteps;
}

Target *HttpTargetProvider::getTarget(int index) {
    return &targets[index][0];
}

std::string HttpTargetProvider::getBaseUrl() {
    if(homeWork_ == "hw3") {
        return "/" + homeWork_ + "/" + basePath_;
    } else if (homeWork_ == "hw9") {
        return "/" + homeWork_ + "/api/hw9/tests";
    } else {
            return "/";
    }
}

bool HttpTargetProvider::load()
{
    std::string testName = getTestName(10);
    std::string fullPath = getBaseUrl() + "/" + testName;
    std::string  rawResponse = downloadFile(fullPath);

    parseTargets(rawResponse);

    return true;
}

std::string HttpTargetProvider::getTestName(const int testNumber)
{
    std::string fullPath = getBaseUrl();
    std::string rawResponse = downloadFile(fullPath);

    json data = json::parse(rawResponse);

    if (testNumber < 1 || testNumber > static_cast<int>(data.size())) {
        LOG("Invalid test number: " << testNumber << " (valid range: 1-" << static_cast<int>(data.size()) << ")");
        return "";
    }

    DEBUG("Test found: " << data[testNumber - 1]["name"].get<std::string>());

    return data[testNumber - 1]["id"].get<std::string>();
}

bool HttpTargetProvider::parseTargets(const std::string &rawResponse)
{
    json data = json::parse(rawResponse);

    targetCount = (int)data["targets"]["targetCount"];
    timeSteps = (int)data["targets"]["timeSteps"];

    targets.resize(targetCount, std::vector<Target>(timeSteps));
    for (int i = 0; i < targetCount; i++) {
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = data["targets"]["targets"][i]["positions"][j]["x"];
            targets[i][j].y = data["targets"]["targets"][i]["positions"][j]["y"];
        }
    }

    return true;
}

std::string HttpTargetProvider::downloadFile(const std::string& fullPath) {

    DEBUG("Downloading data from: " << apiURL_ << fullPath);
    
    httplib::Client cli(apiURL_);
    auto res = cli.Get(fullPath);

    if (res && res->status == 200) {
        std::string data = res->body;
        return data;
    } else {
        LOG("Failed to download targets: " << (res ? res->status : 0) << " URL:" << res->location);
    }

    return "";
}
