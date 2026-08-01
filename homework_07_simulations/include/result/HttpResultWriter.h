#pragma once

#if ENABLE_HTTP

#include <string>
#include <nlohmann/json.hpp>
#include "common/SimStep.h"
#include "interfaces/IResultWriter.h"

// ============ HttpResultWriter ============
class HttpResultWriter : public IResultWriter {
private:
    std::string apiURL_ = "http://cppmiltech.com.ua";
    std::string apiKey_ = "dz12-vX7mK4qT9r2w";
    std::string studentId_;
    std::string testId_;
    int maxRetries_ = 5;
    double retryDelaySec_ = 1.0;
    double connectionTimeoutSec_ = 2.0;
    double readTimeoutSec_ = 2.0;

    bool verifyOnServer(const std::string& testId);

public:
    HttpResultWriter(const std::string& studentId,  const std::string& testId, const std::string& apiUrl = "") {
        if (!apiUrl.empty()) {
            apiURL_ = apiUrl;
        }
        studentId_ = studentId;
        testId_ = testId;
    };

    bool write(const std::vector<SimStep>& steps) override;
    void setApiUrl(const std::string &apiUrl) { apiURL_ = apiUrl; }
    void setApiKey(const std::string& key);
    void setStudentId(const std::string& id);
    void setTestId(const std::string& id) { testId_ = id; }
};

#endif // ENABLE_HTTP
