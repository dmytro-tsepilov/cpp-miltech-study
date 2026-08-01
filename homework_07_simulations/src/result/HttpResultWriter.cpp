#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>

#include <httplib.h>

#ifndef CPPHTTPLIB_OPENSSL_SUPPORT
#define CPPHTTPLIB_OPENSSL_SUPPORT
#endif

#include "common/macros.h"
#include "result/HttpResultWriter.h"

using json = nlohmann::json;

void HttpResultWriter::setApiKey(const std::string& key)
{
    this->apiKey_ = key;
}

void HttpResultWriter::setStudentId(const std::string& id)
{
    this->studentId_ = id;
}

bool HttpResultWriter::verifyOnServer(const std::string& testId)
{
    LOG("Verifying result on server for test: " << testId);

    httplib::Client cli(apiURL_);
    
    // Set timeouts using std::chrono
    cli.set_connection_timeout(std::chrono::duration<double>(connectionTimeoutSec_));
    cli.set_read_timeout(std::chrono::duration<double>(readTimeoutSec_));

    std::string path = "/api/dz12/results/" + testId + "/" + studentId_;
    auto res = cli.Get(path.c_str());

    if (res && res->status == 200) {
        json resp = json::parse(res->body);
        if (resp["found"] == true) {
            LOG("Verification SUCCESS for test " << testId << ": result found on server");
            return true;
        }
    }

    LOG("Verification FAILED for test " << testId << ": status=" << (res ? std::to_string(res->status) : "0"));
    return false;
}

bool HttpResultWriter::write(const std::vector<SimStep>& steps)
{
    LOG("HttpResultWriter::write() starting - sending " << steps.size() << " steps to " << apiURL_);

    // Build JSON payload
    json payload;
    payload["studentId"] = studentId_;
    payload["testId"] = testId_;

    json simulation;
    simulation["totalSteps"] = steps.size();
    simulation["steps"] = json::array();

    std::for_each(steps.begin(), steps.end(), [&simulation](const SimStep& s)
    {
        json step;
        step["position"] = {{"x", s.pos.x}, {"y", s.pos.y}};
        step["direction"] = s.direction;
        step["state"] = s.state;
        step["targetIndex"] = s.targetIdx;
        step["dropPoint"] = {{"x", s.dropPoint.x}, {"y", s.dropPoint.y}};
        step["aimPoint"] = {{"x", s.aimPoint.x}, {"y", s.aimPoint.y}};
        step["predictedTarget"] = {{"x", s.predictedTarget.x}, {"y", s.predictedTarget.y}};
        step["timeSecSinceStart"] = s.timeSecSinceStart;
        simulation["steps"].push_back(step);
    });

    payload["simulation"] = simulation;

    std::string jsonStr = payload.dump(2);
    LOG("Payload JSON: " << jsonStr);

    // Retry loop
    for (int attempt = 1; attempt <= maxRetries_; ++attempt) {
        LOG("Attempt " << attempt << "/" << maxRetries_ << " to send results");

        httplib::Client cli(apiURL_);
        
        // Set timeouts using std::chrono
        cli.set_connection_timeout(std::chrono::duration<double>(connectionTimeoutSec_));
        cli.set_read_timeout(std::chrono::duration<double>(readTimeoutSec_));

        // POST request with x-api-key header
        httplib::Headers headers;
        headers.insert(std::make_pair("x-api-key", apiKey_));
        auto res = cli.Post("/api/dz12/results", headers, jsonStr, "application/json");

        bool success = false;
        bool shouldRetry = false;

        if (res) {
            int statusCode = res->status;
            LOG("Response status: " << statusCode);

            if (statusCode == 200 || statusCode == 201) {
                LOG("POST SUCCESS: HTTP " << statusCode);
                success = true;
            } else if (statusCode == 400) {
                LOG("POST FAILED: HTTP 400 - client error, data validation failed");
                if (res->body.size() > 0) {
                    LOG("Error details: " << res->body);
                }
                shouldRetry = false;
            } else if (statusCode == 401) {
                LOG("POST FAILED: HTTP 401 - authentication failed");
                shouldRetry = false;
            } else if (statusCode == 503) {
                LOG("POST FAILED: HTTP 503 - server unavailable, will retry");
                shouldRetry = true;
            } else {
                // Other error - retry
                LOG("POST FAILED: HTTP " << statusCode << ", will retry");
                shouldRetry = true;
            }
        } else {
            LOG("POST FAILED: connection error, will retry");
            shouldRetry = true;
        }

        if (success) {
            // Verify on server using testId
            bool verified = verifyOnServer(testId_);
            if (verified) {
                LOG("Result verified on server successfully");
                return true;
            } else {
                LOG("Verification failed, but POST was successful - returning true anyway");
                return true;
            }
        }

        if (!shouldRetry) {
            LOG("Not retrying (error code does not support retry)");
            return false;
        }

        // Retry with delay (if attempts remain)
        if (attempt < maxRetries_) {
            LOG("Retrying in " << retryDelaySec_ << " seconds...");
            std::this_thread::sleep_for(std::chrono::duration<double>(retryDelaySec_));
        }
    }

    LOG("All " << maxRetries_ << " attempts failed for HttpResultWriter");
    return false;
}
