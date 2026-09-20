// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT
//
// Test suite for HttpReporter JSON serialization.
// Tests the serialize* methods without requiring network connectivity.

#include <gtest/gtest.h>
#include <string>
#include <regex>
#include <sstream>

// Include the actual implementation for testing
#include "http_reporter/http_reporter.hpp"

// Test 1: Movement report JSON structure
TEST(HttpReporterSerializationTest, MovementReportStructure)
{
    // Create a mock reporter (won't make HTTP calls in these tests)
    HttpReporter reporter("http://localhost:8080");
    
    PerimeterStatus status;
    status.mode = 0;
    status.waypoint_index = 5;
    status.target_x = 10.0;
    status.target_y = 20.0;
    status.current_x = 8.5;
    status.current_y = 18.3;
    status.lateral_error = 0.7;
    status.speed = 1.5f;
    status.mine_detected = false;
    status.timestamp = "1700000000.123456";
    
    // Serialize using the private method (access via public interface)
    std::string json = reporter.serializeMovementReport(status);
    
    // Verify JSON contains all expected fields
    EXPECT_TRUE(json.find("\"mode\":0") != std::string::npos);
    EXPECT_TRUE(json.find("\"waypoint_index\":5") != std::string::npos);
    EXPECT_TRUE(json.find("\"target_x\":") != std::string::npos);
    EXPECT_TRUE(json.find("\"target_y\":") != std::string::npos);
    EXPECT_TRUE(json.find("\"current_x\":") != std::string::npos);
    EXPECT_TRUE(json.find("\"current_y\":") != std::string::npos);
    EXPECT_TRUE(json.find("\"lateral_error\":") != std::string::npos);
    EXPECT_TRUE(json.find("\"speed\":") != std::string::npos);
    EXPECT_TRUE(json.find("\"mine_detected\":false") != std::string::npos);
    EXPECT_TRUE(json.find("\"timestamp\":\"1700000000.123456\"") != std::string::npos);
}

// Test 2: Mine detection JSON structure
TEST(HttpReporterSerializationTest, MineDetectionStructure)
{
    HttpReporter reporter("http://localhost:8080");
    
    MineDetection detection;
    detection.mine_id = 42;
    detection.x = 15.5;
    detection.y = 25.3;
    detection.type = "anti-tank";
    detection.confidence = 0.95f;
    detection.detected_at = "1700000000.654321";
    
    std::string json = reporter.serializeMineDetection(detection);
    
    EXPECT_TRUE(json.find("\"mine_id\":42") != std::string::npos);
    EXPECT_TRUE(json.find("\"x\":") != std::string::npos);
    EXPECT_TRUE(json.find("\"y\":") != std::string::npos);
    EXPECT_TRUE(json.find("\"type\":\"anti-tank\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"confidence\":") != std::string::npos);
    EXPECT_TRUE(json.find("\"detected_at\":\"1700000000.654321\"") != std::string::npos);
}

// Test 3: Clearance report JSON structure
TEST(HttpReporterSerializationTest, ClearanceReportStructure)
{
    HttpReporter reporter("http://localhost:8080");
    
    ClearanceReport report;
    report.mine_id = 42;
    report.x = 15.5;
    report.y = 25.3;
    report.method = "disposal";
    report.success = true;
    report.details = "Mine neutralized successfully";
    report.timestamp = "1700000100.111111";
    
    std::string json = reporter.serializeClearanceReport(report);
    
    EXPECT_TRUE(json.find("\"mine_id\":42") != std::string::npos);
    EXPECT_TRUE(json.find("\"method\":\"disposal\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"success\":true") != std::string::npos);
    EXPECT_TRUE(json.find("\"details\":\"Mine neutralized successfully\"") != std::string::npos);
}

// Test 4: Mission summary JSON structure
TEST(HttpReporterSerializationTest, MissionSummaryStructure)
{
    HttpReporter reporter("http://localhost:8080");
    
    MissionSummary summary;
    summary.scenario_name = "training_ground";
    summary.result = "SUCCESS";
    summary.reason = "All waypoints completed and all mines cleared";
    summary.total_waypoints = 4;
    summary.waypoints_completed = 4;
    summary.mines_detected = 3;
    summary.mines_cleared = 3;
    summary.mission_duration = 120.5;
    summary.coverage_percent = 100.0;
    summary.start_time = "1700000000";
    summary.end_time = "1700000120";
    
    std::string json = reporter.serializeMissionSummary(summary);
    
    EXPECT_TRUE(json.find("\"scenario_name\":\"training_ground\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"result\":\"SUCCESS\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"total_waypoints\":4") != std::string::npos);
    EXPECT_TRUE(json.find("\"waypoints_completed\":4") != std::string::npos);
    EXPECT_TRUE(json.find("\"mines_detected\":3") != std::string::npos);
    EXPECT_TRUE(json.find("\"mines_cleared\":3") != std::string::npos);
    EXPECT_TRUE(json.find("\"coverage_percent\":") != std::string::npos);
}

// Test 5: JSON is valid (basic syntax check)
TEST(HttpReporterSerializationTest, MovementReportIsValidJson)
{
    HttpReporter reporter("http://localhost:8080");
    
    PerimeterStatus status;
    status.mode = 1;
    status.waypoint_index = 0;
    status.target_x = 0.0;
    status.target_y = 0.0;
    status.current_x = 0.0;
    status.current_y = 0.0;
    status.lateral_error = 0.0;
    status.speed = 0.0f;
    status.mine_detected = false;
    status.timestamp = "0.0";
    
    std::string json = reporter.serializeMovementReport(status);
    
    // Basic JSON validation: starts with { and ends with }
    EXPECT_TRUE(json.front() == '{');
    EXPECT_TRUE(json.back() == '}');
    
    // Should contain balanced braces
    int brace_count = 0;
    for (char c : json) {
        if (c == '{') brace_count++;
        if (c == '}') brace_count--;
    }
    EXPECT_EQ(brace_count, 0);
}

// Test 6: Mine detection with different types
TEST(HttpReporterSerializationTest, MineDetectionTypes)
{
    HttpReporter reporter("http://localhost:8080");
    
    // Test anti-personnel type
    MineDetection det1;
    det1.mine_id = 1;
    det1.x = 5.0;
    det1.y = 10.0;
    det1.type = "anti-personnel";
    det1.confidence = 0.8f;
    det1.detected_at = "0.0";
    
    std::string json1 = reporter.serializeMineDetection(det1);
    EXPECT_TRUE(json1.find("\"type\":\"anti-personnel\"") != std::string::npos);
    
    // Test unknown type
    MineDetection det2;
    det2.mine_id = 2;
    det2.x = 15.0;
    det2.y = 25.0;
    det2.type = "unknown";
    det2.confidence = 0.5f;
    det2.detected_at = "0.0";
    
    std::string json2 = reporter.serializeMineDetection(det2);
    EXPECT_TRUE(json2.find("\"type\":\"unknown\"") != std::string::npos);
}

// Test 7: BuildUrl with leading slash
TEST(HttpReporterSerializationTest, BuildUrlWithSlash)
{
    HttpReporter reporter("http://localhost:8080");
    
    std::string url = reporter.buildUrl("/api/v1/movement");
    EXPECT_EQ(url, "http://localhost:8080/api/v1/movement");
}

// Test 8: BuildUrl without leading slash
TEST(HttpReporterSerializationTest, BuildUrlWithoutSlash)
{
    HttpReporter reporter("http://localhost:8080");
    
    std::string url = reporter.buildUrl("api/v1/movement");
    EXPECT_EQ(url, "http://localhost:8080/api/v1/movement");
}

// Test 9: Empty path handling
TEST(HttpReporterSerializationTest, BuildUrlEmptyPath)
{
    HttpReporter reporter("http://localhost:8080");
    
    std::string url = reporter.buildUrl("");
    EXPECT_EQ(url, "http://localhost:8080/");
}

// Test 10: Movement report with mine detected flag
TEST(HttpReporterSerializationTest, MineDetectedTrue)
{
    HttpReporter reporter("http://localhost:8080");
    
    PerimeterStatus status;
    status.mode = 2;  // HOLD mode
    status.waypoint_index = 3;
    status.target_x = 20.0;
    status.target_y = 20.0;
    status.current_x = 18.5;
    status.current_y = 19.2;
    status.lateral_error = 0.7;
    status.speed = 1.5f;
    status.mine_detected = true;  // Changed to true
    status.timestamp = "1700000000.123456";
    
    std::string json = reporter.serializeMovementReport(status);
    
    EXPECT_TRUE(json.find("\"mine_detected\":true") != std::string::npos);
    EXPECT_TRUE(json.find("\"mode\":2") != std::string::npos);
}

// Test 11: Clearance report with failure
TEST(HttpReporterSerializationTest, ClearanceReportFailure)
{
    HttpReporter reporter("http://localhost:8080");
    
    ClearanceReport report;
    report.mine_id = 99;
    report.x = 50.0;
    report.y = 60.0;
    report.method = "marking";
    report.success = false;  // Failed
    report.details = "Clearance failed - unstable ground";
    report.timestamp = "1700000200.000000";
    
    std::string json = reporter.serializeClearanceReport(report);
    
    EXPECT_TRUE(json.find("\"success\":false") != std::string::npos);
    EXPECT_TRUE(json.find("\"method\":\"marking\"") != std::string::npos);
}

// Test 12: Mission summary with failure result
TEST(HttpReporterSerializationTest, MissionSummaryFailure)
{
    HttpReporter reporter("http://localhost:8080");
    
    MissionSummary summary;
    summary.scenario_name = "training_ground";
    summary.result = "FAILED";
    summary.reason = "Mine detection failed - sensor malfunction";
    summary.total_waypoints = 4;
    summary.waypoints_completed = 2;
    summary.mines_detected = 1;
    summary.mines_cleared = 0;
    summary.mission_duration = 60.0;
    summary.coverage_percent = 50.0;
    summary.start_time = "1700000000";
    summary.end_time = "1700000060";
    
    std::string json = reporter.serializeMissionSummary(summary);
    
    EXPECT_TRUE(json.find("\"result\":\"FAILED\"") != std::string::npos);
    EXPECT_TRUE(json.find("\"coverage_percent\":50.0") != std::string::npos);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
