#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "common/SimStep.h"
#include "interfaces/IResultWriter.h"

// ============ SocketResultWriter ============

class SocketResultWriter : public IResultWriter {
private:
    std::string host_ = "cppmiltech.com.ua";
    int port_ = 80;  // default HTTP port
    std::string apiKey_ = "dz12-vX7mK4qT9r2w";
    std::string studentId_;
    std::string testId_;
    int maxRetries_ = 5;
    double retryDelaySec_ = 1.0;
    int connectionTimeoutMs_ = 2000;
    int readTimeoutMs_ = 2000;

    // Raw socket file descriptor
    int sockfd_ = -1;

    // Connect to the server using raw TCP socket
    bool connect();

    // Close the socket connection
    void disconnect();

    // Send data over the socket
    bool sendData(const std::string& data);

    // Receive response from the socket
    std::string recvData();

    // Parse HTTP response to get status code
    int parseHttpResponse(const std::string& response);

    // Build HTTP POST request string
    std::string buildPostRequest(const std::string& path, const std::string& body);

    // Build HTTP GET request string
    std::string buildGetRequest(const std::string& path);

    // Verify result on server using testId
    bool verifyOnServer(const std::string& testId);

    // Parse URL to extract host and port
    void parseUrl(const std::string& url);

public:
    SocketResultWriter(const std::string& studentId, const std::string& testId, 
                       const std::string& host = "", int port = 80);
    ~SocketResultWriter() override;

    bool write(const std::vector<SimStep>& steps) override;
    
    void setHost(const std::string& host) { host_ = host; }
    void setPort(int port) { port_ = port; }
    void setApiKey(const std::string& key);
    void setStudentId(const std::string& id);
    void setTestId(const std::string& id) { testId_ = id; }
    void setMaxRetries(int retries) { maxRetries_ = retries; }
    void setRetryDelay(double delaySec) { retryDelaySec_ = delaySec; }
    void setConnectionTimeoutMs(int ms) { connectionTimeoutMs_ = ms; }
    void setReadTimeoutMs(int ms) { readTimeoutMs_ = ms; }
};
