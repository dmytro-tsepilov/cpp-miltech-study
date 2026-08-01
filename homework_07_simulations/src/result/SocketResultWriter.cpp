#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>

#include "common/macros.h"
#include "result/SocketResultWriter.h"

using json = nlohmann::json;

// ============ URL Parsing ============

void SocketResultWriter::parseUrl(const std::string& url)
{
    std::string temp = url;
    
    // Remove protocol prefix if present (e.g., "http://" or "https://")
    size_t protoStart = temp.find("://");
    if (protoStart != std::string::npos) {
        temp.erase(0, protoStart + 3);
    }
    
    // Extract host (remove port if present)
    size_t portStart = temp.find(':');
    if (portStart != std::string::npos) {
        host_ = temp.substr(0, portStart);
        std::string portStr = temp.substr(portStart + 1);
        // Remove trailing path if any
        size_t pathStart = portStr.find('/');
        if (pathStart != std::string::npos) {
            portStr = portStr.substr(0, pathStart);
        }
        try {
            port_ = std::stoi(portStr);
        } catch (...) {
            LOG("Invalid port in URL: " << portStr << ", using default: " << port_);
        }
    } else {
        // No port, check for path
        size_t pathStart = temp.find('/');
        if (pathStart != std::string::npos) {
            host_ = temp.substr(0, pathStart);
        } else {
            host_ = temp;
        }
    }
    
    // Set default port based on protocol if no explicit port was found in URL
    if (portStart == std::string::npos) {
        // No port specified in URL, use protocol-based default
        if (url.find("https://") == 0) {
            port_ = 443;
        } else if (url.find("http://") == 0) {
            port_ = 80;
        }
        // If no protocol found, keep the constructor's default port
    }
    
    LOG("Parsed URL: host=" << host_ << ", port=" << port_);
}

// ============ Constructor / Destructor ============

SocketResultWriter::SocketResultWriter(const std::string& studentId, const std::string& testId,
                                       const std::string& host, int port)
{
    if (!host.empty()) {
        // Check if it's a full URL with protocol
        if (host.find("://") != std::string::npos) {
            parseUrl(host);
        } else {
            host_ = host;
        }
    }
    port_ = port;
    studentId_ = studentId;
    testId_ = testId;
}

SocketResultWriter::~SocketResultWriter()
{
    if (sockfd_ >= 0) {
        ::close(sockfd_);
        sockfd_ = -1;
    }
}

// ============ Socket Connection Helpers ============

bool SocketResultWriter::connect()
{
    if (sockfd_ >= 0) {
        ::close(sockfd_);
        sockfd_ = -1;
    }

    // Resolve hostname
    struct addrinfo hints, *result, *ptr;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;      // IPv4
    hints.ai_socktype = SOCK_STREAM; // TCP
    hints.ai_protocol = IPPROTO_TCP;

    std::string portStr = std::to_string(port_);
    int ret = getaddrinfo(host_.c_str(), portStr.c_str(), &hints, &result);
    if (ret != 0) {
        LOG("getaddrinfo failed for " << host_ << ":" << port_ << ": " << gai_strerror(ret));
        return false;
    }

    // Create socket and connect
    sockfd_ = -1;
    for (ptr = result; ptr != nullptr; ptr = ptr->ai_next) {
        sockfd_ = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (sockfd_ < 0) {
            continue;
        }

        // Set connection timeout
        struct timeval tv;
        tv.tv_sec = connectionTimeoutMs_ / 1000;
        tv.tv_usec = (connectionTimeoutMs_ % 1000) * 1000;
        setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        if (::connect(sockfd_, ptr->ai_addr, ptr->ai_addrlen) == 0) {
            LOG("Connected to " << host_ << ":" << port_);
            freeaddrinfo(result);
            return true;
        }

        LOG("connect failed: " << strerror(errno));
        ::close(sockfd_);
        sockfd_ = -1;
    }

    freeaddrinfo(result);
    LOG("Failed to connect to " << host_ << ":" << port_);
    return false;
}

void SocketResultWriter::disconnect()
{
    if (sockfd_ >= 0) {
        ::shutdown(sockfd_, SHUT_RDWR);
        ::close(sockfd_);
        sockfd_ = -1;
    }
}

bool SocketResultWriter::sendData(const std::string& data)
{
    if (sockfd_ < 0) {
        LOG("Socket not connected, cannot send data");
        return false;
    }

    size_t totalSent = 0;
    size_t dataSize = data.size();

    while (totalSent < dataSize) {
        ssize_t sent = ::send(sockfd_, data.c_str() + totalSent, dataSize - totalSent, 0);
        if (sent <= 0) {
            LOG("send() failed: " << strerror(errno) << " (sent " << totalSent << "/" << dataSize << ")");
            return false;
        }
        totalSent += sent;
    }

    LOG("Sent " << totalSent << " bytes");
    return true;
}

std::string SocketResultWriter::recvData()
{
    if (sockfd_ < 0) {
        LOG("Socket not connected, cannot receive data");
        return "";
    }

    std::string response;
    char buffer[4096];

    // Set read timeout
    struct timeval tv;
    tv.tv_sec = readTimeoutMs_ / 1000;
    tv.tv_usec = (readTimeoutMs_ % 1000) * 1000;
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (true) {
        ssize_t received = ::recv(sockfd_, buffer, sizeof(buffer), 0);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Timeout - check if we got any data
                if (!response.empty()) {
                    LOG("Received " << response.size() << " bytes before timeout");
                    break;
                }
                LOG("Receive timeout with no data");
                return "";
            }
            LOG("recv() failed: " << strerror(errno));
            return "";
        }
        if (received == 0) {
            // Server closed connection
            LOG("Server closed connection, received " << response.size() << " bytes total");
            break;
        }
        response.append(buffer, received);
    }

    return response;
}

int SocketResultWriter::parseHttpResponse(const std::string& response)
{
    // Look for HTTP status line: "HTTP/1.1 200 OK"
    size_t pos = response.find(" ");
    if (pos == std::string::npos) {
        LOG("No HTTP status line found");
        return 0;
    }

    size_t codeStart = pos + 1;
    size_t codeEnd = response.find(" ", codeStart);
    if (codeEnd == std::string::npos) {
        codeEnd = response.find("\r", codeStart);
    }
    if (codeEnd == std::string::npos) {
        codeEnd = response.find("\n", codeStart);
    }

    if (codeEnd > codeStart) {
        std::string statusCodeStr = response.substr(codeStart, codeEnd - codeStart);
        try {
            return std::stoi(statusCodeStr);
        } catch (...) {
            LOG("Failed to parse status code: " << statusCodeStr);
            return 0;
        }
    }

    return 0;
}

std::string SocketResultWriter::buildPostRequest(const std::string& path, const std::string& body)
{
    std::ostringstream request;
    request << "POST " << path << " HTTP/1.1\r\n";
    request << "Host: " << host_ << "\r\n";
    request << "Content-Type: application/json\r\n";
    request << "Content-Length: " << body.size() << "\r\n";
    request << "x-api-key: " << apiKey_ << "\r\n";
    request << "Connection: close\r\n";
    request << "\r\n";

    LOG("Raw Request: " << request.str());
    request << body;

    return request.str();
}

std::string SocketResultWriter::buildGetRequest(const std::string& path)
{
    std::ostringstream request;
    request << "GET " << path << " HTTP/1.1\r\n";
    request << "Host: " << host_ << "\r\n";
    request << "x-api-key: " << apiKey_ << "\r\n";
    request << "Connection: close\r\n";
    request << "\r\n";

    return request.str();
}

// ============ Verification ============

bool SocketResultWriter::verifyOnServer(const std::string& testId)
{
    LOG("Verifying result on server for test: " << testId);

    if (!connect()) {
        LOG("Failed to connect for verification");
        return false;
    }

    std::string path = "/api/dz12/results/" + testId + "/" + studentId_;
    std::string request = buildGetRequest(path);

    bool success = false;
    if (sendData(request)) {
        std::string response = recvData();
        int statusCode = parseHttpResponse(response);

        if (statusCode == 200) {
            // Extract JSON body from raw HTTP response (split on \r\n\r\n or \n\n)
            std::string jsonBody;
            size_t headerEnd = response.find("\r\n\r\n");
            if (headerEnd != std::string::npos) {
                jsonBody = response.substr(headerEnd + 4);
            } else {
                headerEnd = response.find("\n\n");
                if (headerEnd != std::string::npos) {
                    jsonBody = response.substr(headerEnd + 2);
                }
            }

            LOG("Verification response: " << response);
            if (!jsonBody.empty()) {
                try {
                    json resp = json::parse(jsonBody);
                    if (resp["found"] == true) {
                        LOG("Verification SUCCESS for test " << testId << ": result found on server");
                        success = true;
                    }
                } catch (const std::exception& e) {
                    LOG("Failed to parse verification response: " << e.what());
                }
            } else {
                LOG("Verification failed: empty JSON body in response");
            }
        }
    }

    disconnect();
    return success;
}

// ============ Main Write Method ============

bool SocketResultWriter::write(const std::vector<SimStep>& steps)
{
    LOG("SocketResultWriter::write() starting - sending " << steps.size() << " steps to " 
        << host_ << ":" << port_);

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

        // Connect
        if (!connect()) {
            LOG("Failed to connect on attempt " << attempt);
            if (attempt < maxRetries_) {
                LOG("Retrying in " << retryDelaySec_ << " seconds...");
                std::this_thread::sleep_for(std::chrono::duration<double>(retryDelaySec_));
            }
            continue;
        }

        // Build and send POST request
        std::string path = "/api/dz12/results";
        std::string request = buildPostRequest(path, jsonStr);

        bool success = false;
        bool shouldRetry = false;

        if (sendData(request)) {
            std::string response = recvData();
            int statusCode = parseHttpResponse(response);
            LOG("Response status: " << statusCode);

            if (statusCode == 200 || statusCode == 201) {
                LOG("POST SUCCESS: HTTP " << statusCode);
                success = true;
            } else if (statusCode == 400) {
                LOG("POST FAILED: HTTP 400 - client error, data validation failed");
                if (response.size() > 0) {
                    LOG("Error details: " << response);
                }
                shouldRetry = false;
            } else if (statusCode == 401) {
                LOG("POST FAILED: HTTP 401 - authentication failed");
                shouldRetry = false;
            } else if (statusCode == 503) {
                LOG("POST FAILED: HTTP 503 - server unavailable, will retry");
                shouldRetry = true;
            } else {
                LOG("POST FAILED: HTTP " << statusCode << ", will retry");
                shouldRetry = true;
            }
        } else {
            LOG("POST FAILED: send error, will retry");
            shouldRetry = true;
        }

        // Disconnect after each attempt
        disconnect();

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

    LOG("All " << maxRetries_ << " attempts failed for SocketResultWriter");
    return false;
}
