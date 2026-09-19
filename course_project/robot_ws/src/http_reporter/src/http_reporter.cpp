#include "http_reporter/http_reporter.hpp"

#include <curl/curl.h>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <iostream>

// Callback for curl write
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t total_size = size * nmemb;
    auto* output = static_cast<std::string*>(userp);
    output->append(static_cast<char*>(contents), total_size);
    return total_size;
}

HttpReporter::HttpReporter(const std::string& endpoint, int timeout_ms)
    : endpoint_(endpoint), timeout_ms_(timeout_ms) {
    curl_global_init(CURL_GLOBAL_DEFAULT);
}

HttpReporter::~HttpReporter() {
    curl_global_cleanup();
}

std::string HttpReporter::buildUrl(const std::string& path) const {
    if (path.empty() || path[0] != '/') {
        return endpoint_ + "/" + path;
    }
    return endpoint_ + path;
}

bool HttpReporter::postJson(const std::string& path, const std::string& json_data) {
    std::string url = buildUrl(path);
    
    CURL* curl = curl_easy_init();
    if (!curl) {
        last_error_ = "Failed to initialize curl";
        return false;
    }
    
    std::string response_data;
    CURLcode res;
    
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(json_data.size()));
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms_);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, timeout_ms_ / 2);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    
    res = curl_easy_perform(curl);
    
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
        last_error_ = std::string("curl_easy_perform failed: ") + curl_easy_strerror(res);
        failure_count_++;
        std::cerr << "[HttpReporter] HTTP POST failed to " << url << ": " << last_error_ << std::endl;
        return false;
    }
    
    success_count_++;
    std::cout << "[HttpReporter] HTTP POST success to " << url << std::endl;
    return true;
}

std::string HttpReporter::serializeMovementReport(const PerimeterStatus& status) {
    std::ostringstream json;
    json << "{";
    json << "\"mode\":" << static_cast<int>(status.mode) << ",";
    json << "\"waypoint_index\":" << status.waypoint_index << ",";
    json << "\"target_x\":" << std::fixed << std::setprecision(6) << status.target_x << ",";
    json << "\"target_y\":" << std::fixed << std::setprecision(6) << status.target_y << ",";
    json << "\"current_x\":" << std::fixed << std::setprecision(6) << status.current_x << ",";
    json << "\"current_y\":" << std::fixed << std::setprecision(6) << status.current_y << ",";
    json << "\"lateral_error\":" << std::fixed << std::setprecision(6) << status.lateral_error << ",";
    json << "\"speed\":" << std::fixed << std::setprecision(2) << static_cast<double>(status.speed) << ",";
    json << "\"mine_detected\":" << (status.mine_detected ? "true" : "false") << ",";
    json << "\"timestamp\":\"" << status.timestamp << "\"";
    json << "}";
    return json.str();
}

std::string HttpReporter::serializeMineDetection(const MineDetection& detection) {
    std::ostringstream json;
    json << "{";
    json << "\"mine_id\":" << detection.mine_id << ",";
    json << "\"x\":" << std::fixed << std::setprecision(6) << detection.x << ",";
    json << "\"y\":" << std::fixed << std::setprecision(6) << detection.y << ",";
    json << "\"type\":\"" << detection.type << "\",";
    json << "\"confidence\":" << std::fixed << std::setprecision(2) << static_cast<double>(detection.confidence) << ",";
    json << "\"detected_at\":\"" << detection.detected_at << "\"";
    json << "}";
    return json.str();
}

std::string HttpReporter::serializeClearanceReport(const ClearanceReport& report) {
    std::ostringstream json;
    json << "{";
    json << "\"mine_id\":" << report.mine_id << ",";
    json << "\"x\":" << std::fixed << std::setprecision(6) << report.x << ",";
    json << "\"y\":" << std::fixed << std::setprecision(6) << report.y << ",";
    json << "\"method\":\"" << report.method << "\",";
    json << "\"success\":" << (report.success ? "true" : "false") << ",";
    json << "\"details\":\"" << report.details << "\",";
    json << "\"timestamp\":\"" << report.timestamp << "\"";
    json << "}";
    return json.str();
}

std::string HttpReporter::serializeMissionSummary(const MissionSummary& summary) {
    std::ostringstream json;
    json << "{";
    json << "\"scenario_name\":\"" << summary.scenario_name << "\",";
    json << "\"result\":\"" << summary.result << "\",";
    json << "\"reason\":\"" << summary.reason << "\",";
    json << "\"total_waypoints\":" << summary.total_waypoints << ",";
    json << "\"waypoints_completed\":" << summary.waypoints_completed << ",";
    json << "\"mines_detected\":" << summary.mines_detected << ",";
    json << "\"mines_cleared\":" << summary.mines_cleared << ",";
    json << "\"mission_duration\":" << std::fixed << std::setprecision(6) << summary.mission_duration << ",";
    json << "\"coverage_percent\":" << std::fixed << std::setprecision(2) << summary.coverage_percent << ",";
    json << "\"start_time\":\"" << summary.start_time << "\",";
    json << "\"end_time\":\"" << summary.end_time << "\"";
    json << "}";
    return json.str();
}

bool HttpReporter::sendMovementReport(const PerimeterStatus& status) {
    std::string json = serializeMovementReport(status);
    return postJson("/api/v1/movement", json);
}

bool HttpReporter::sendMineDetection(const MineDetection& detection) {
    std::string json = serializeMineDetection(detection);
    return postJson("/api/v1/mine/detected", json);
}

bool HttpReporter::sendClearanceReport(const ClearanceReport& report) {
    std::string json = serializeClearanceReport(report);
    return postJson("/api/v1/mine/cleared", json);
}

bool HttpReporter::sendMissionSummary(const MissionSummary& summary) {
    std::string json = serializeMissionSummary(summary);
    return postJson("/api/v1/mission/summary", json);
}
