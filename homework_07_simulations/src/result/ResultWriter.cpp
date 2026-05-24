#include <fstream>
#include <string>
#include <cstring>

#include "common/macros.h"
#include "result/ResultWriter.h"

using json = nlohmann::json;

// ============ JsonResultWriter Implementation ============

bool JsonResultWriter::write(SimStep* steps, int stepCount)
{
    std::ofstream outFile(folderPath + filename);
    if (!outFile.is_open())
    {
        LOG("Error opening " << folderPath + filename << " for writing");
        return false;
    }

    LOG("Writing " << stepCount << " steps to " << folderPath + filename << " (JSON format)");

    json out;
    out["totalSteps"] = stepCount;
    out["steps"] = json::array();

    for (int i = 0; i < stepCount; i++)
    {
        json step;
        step["position"] = {{"x", steps[i].pos.x}, {"y", steps[i].pos.y}};
        step["direction"] = steps[i].direction;
        step["state"] = steps[i].state;
        step["targetIndex"] = steps[i].targetIdx;
        step["dropPoint"] = {{"x", steps[i].dropPoint.x}, {"y", steps[i].dropPoint.y}};
        step["aimPoint"] = {{"x", steps[i].aimPoint.x}, {"y", steps[i].aimPoint.y}};
        step["predictedTarget"] = {{"x", steps[i].predictedTarget.x}, {"y", steps[i].predictedTarget.y}};
        out["steps"].push_back(step);
    }

    outFile << out.dump(2);  // 2 = indent for readability
    outFile.close();

    LOG("JSON simulation completed: " << stepCount << " steps written to " << folderPath + filename);

    return true;
}

void JsonResultWriter::setFolderPath(const std::string folderPath)
{
    this->folderPath = folderPath;
}

void JsonResultWriter::setFilename(const std::string &filename)
{
    this->filename = filename;
}

// ============ ApiResultWriter Implementation ============

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
bool ApiResultWriter::write(SimStep* steps, int stepCount)
{
    // TODO: Implement API submission logic
    // TODO: Handle HTTP POST request with authentication
    // TODO: Serialize SimStep data to JSON payload
    LOG("ApiResultWriter::write() - not implemented yet");
    LOG("API URL: " << apiUrl);
    LOG("Auth Token: " << (authToken.empty() ? "(not set)" : "(set)"));
    return false;
}
#pragma clang diagnostic pop

void ApiResultWriter::setApiUrl(const std::string &url)
{
    this->apiUrl = url;
}

void ApiResultWriter::setAuthToken(const std::string &token)
{
    this->authToken = token;
}

// ============ DatabaseResultWriter Implementation ============

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
bool DatabaseResultWriter::write(SimStep* steps, int stepCount)
{
    // TODO: Implement database insertion logic
    // TODO: Handle connection management
    // TODO: Prepare SQL INSERT statements
    LOG("DatabaseResultWriter::write() - not implemented yet");
    LOG("Connection String: " << connectionString);
    LOG("Table Name: " << tableName);
    return false;
}
#pragma clang diagnostic pop

void DatabaseResultWriter::setConnectionString(const std::string &connectionString)
{
    this->connectionString = connectionString;
}

void DatabaseResultWriter::setTableName(const std::string &tableName)
{
    this->tableName = tableName;
}
