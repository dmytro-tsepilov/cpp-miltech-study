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

Target HttpTargetProvider::getTarget(int index) {
    if (index < 0 || index >= targetCount) {
        return Target{};
    }
    return Target{ targets[index][0], {0, 0} };
}

std::string HttpTargetProvider::getBaseUrl() {
    if (homeWork_ == "hw3") {
        return "/" + homeWork_ + "/" + basePath_;
    } else if (homeWork_ == "hw9") {
        return "/hw3/api/hw9/tests";
    } else {
        return "/";
    }
}

bool HttpTargetProvider::load()
{
    if (testNumber_) {
        testName_ = getTestName(testNumber_);
    }

    if (testName_.empty()) {
        LOG("Test name is empty. Cannot load targets.");
        return false;
    }

    std::string rawResponse;
    int configStatus = downloadFile(getBaseUrl() + "/" + testName_, rawResponse);
    if (configStatus != 200) {
        return false;
    }

    parseTargets(rawResponse);

    return true;
}

std::string HttpTargetProvider::getTestName(const int testNumber)
{
    std::string rawResponse;
    downloadFile(getBaseUrl(), rawResponse);

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

    targets.resize(targetCount, std::vector<Coord>(timeSteps));
    for (int i = 0; i < targetCount; i++) {
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = data["targets"]["targets"][i]["positions"][j]["x"];
            targets[i][j].y = data["targets"]["targets"][i]["positions"][j]["y"];
        }
    }

    return true;
}

int HttpTargetProvider::downloadFile(const std::string& fullPath, std::string &rawResponse) {
    httplib::Client cli(apiURL_);
    auto res = cli.Get(fullPath);

    if (res && res->status == 200) {
        std::string data = res->body;
        DEBUG("[" << typeid(*this).name() << "] Download completed from: size=" << data.size() << " url=" << apiURL_ << fullPath);
        rawResponse = data;
    } else {
        LOG("[" << typeid(*this).name() << "] Failed to download targets from " << apiURL_ << fullPath << ": Error: " << (res ? res->status : 0));
    }

    return res ? res->status : -1;
}
