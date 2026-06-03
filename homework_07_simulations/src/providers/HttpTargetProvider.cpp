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
    return targets[index];
}

bool HttpTargetProvider::load()
{
    std::string rawResponse;
    rawResponse = downloadFile(apiURL_, testName_);

    parseTargets(rawResponse);

    return true;
}

bool HttpTargetProvider::parseTargets(const std::string &rawResponse)
{
    json data = json::parse(rawResponse);

    targetCount = (int)data["targets"]["targetCount"];
    timeSteps = (int)data["targets"]["timeSteps"];

    targets = new Target*[targetCount];
    for (int i = 0; i < targetCount; i++) {
        targets[i] = new Target[timeSteps];
        for (int j = 0; j < timeSteps; j++) {
            targets[i][j].x = data["targets"]["targets"][i]["positions"][j]["x"];
            targets[i][j].y = data["targets"]["targets"][i]["positions"][j]["y"];
        }
    }

    return true;
}

std::string HttpTargetProvider::downloadFile(const std::string& baseUrl, const std::string& path) {
    httplib::Client cli(baseUrl);
    auto res = cli.Get(path);

    if (res && res->status == 200) {
        std::string data = res->body;
        return data;
    }

    return "";
}
