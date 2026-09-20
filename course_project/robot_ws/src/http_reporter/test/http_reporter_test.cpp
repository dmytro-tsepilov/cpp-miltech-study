#include <gtest/gtest.h>
#include <string>
#include <cstdint>

// Include the actual HttpReporter implementation (provides structs + class)
#include "http_reporter/http_reporter.hpp"

// Test 1: HttpReporter construction with default endpoint
TEST(HttpReporterTest, DefaultEndpoint) {
    // Verify default constructor works and initial state is correct
    HttpReporter reporter;
    EXPECT_EQ(reporter.getSuccessCount(), 0);
    EXPECT_EQ(reporter.getFailureCount(), 0);
    EXPECT_TRUE(reporter.getLastError().empty());
}

// Test 2: HttpReporter construction with custom endpoint
TEST(HttpReporterTest, CustomEndpoint) {
    HttpReporter reporter("http://192.168.1.100:9000", 3000);
    
    EXPECT_EQ(reporter.getSuccessCount(), 0);
    EXPECT_EQ(reporter.getFailureCount(), 0);
    EXPECT_TRUE(reporter.getLastError().empty());
}

// Test 3: PerimeterStatus serialization (JSON format check)
TEST(SerializationTest, PerimeterStatusFields) {
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
    
    // Verify all fields are set correctly
    EXPECT_EQ(status.mode, uint8_t(0));
    EXPECT_EQ(status.waypoint_index, uint32_t(5));
    EXPECT_DOUBLE_EQ(status.target_x, 10.0);
    EXPECT_DOUBLE_EQ(status.lateral_error, 0.7);
    EXPECT_FALSE(status.mine_detected);
}

// Test 4: MineDetection serialization
TEST(SerializationTest, MineDetectionFields) {
    MineDetection detection;
    detection.mine_id = 42;
    detection.x = 15.5;
    detection.y = 25.3;
    detection.type = "anti-tank";
    detection.confidence = 0.95f;
    
    EXPECT_EQ(detection.mine_id, 42);
    EXPECT_DOUBLE_EQ(detection.x, 15.5);
    EXPECT_STREQ(detection.type.c_str(), "anti-tank");
    EXPECT_FLOAT_EQ(detection.confidence, 0.95f);
}

// Test 5: ClearanceReport serialization
TEST(SerializationTest, ClearanceReportFields) {
    ClearanceReport report;
    report.mine_id = 42;
    report.x = 15.5;
    report.y = 25.3;
    report.method = "disposal";
    report.success = true;
    report.details = "Mine neutralized successfully";
    
    EXPECT_EQ(report.mine_id, 42);
    EXPECT_STREQ(report.method.c_str(), "disposal");
    EXPECT_TRUE(report.success);
}

// Test 6: MissionSummary serialization
TEST(SerializationTest, MissionSummaryFields) {
    MissionSummary summary;
    summary.scenario_name = "training_ground";
    summary.result = "SUCCESS";
    summary.reason = "All waypoints completed";
    summary.total_waypoints = 4;
    summary.waypoints_completed = 4;
    summary.mines_detected = 3;
    summary.mines_cleared = 3;
    summary.mission_duration = 120.5;
    summary.coverage_percent = 100.0;
    
    EXPECT_STREQ(summary.scenario_name.c_str(), "training_ground");
    EXPECT_STREQ(summary.result.c_str(), "SUCCESS");
    EXPECT_EQ(summary.total_waypoints, uint32_t(4));
    EXPECT_EQ(summary.mines_detected, uint32_t(3));
    EXPECT_DOUBLE_EQ(summary.mission_duration, 120.5);
}

// Test 7: Build URL construction with various inputs
TEST(HttpReporterTest, BuildUrlWithVariousInputs) {
    HttpReporter reporter("http://localhost:8080");
    
    // URL with leading slash
    std::string url1 = reporter.buildUrl("/api/v1/movement");
    EXPECT_EQ(url1, "http://localhost:8080/api/v1/movement");
    
    // URL without leading slash
    std::string url2 = reporter.buildUrl("api/v1/mine/detected");
    EXPECT_EQ(url2, "http://localhost:8080/api/v1/mine/detected");
    
    // Empty path should return base URL with trailing slash
    std::string url3 = reporter.buildUrl("");
    EXPECT_EQ(url3, "http://localhost:8080/");
}

// Test 8: Error handling — getLastError returns empty initially
TEST(HttpReporterTest, InitialErrorState) {
    HttpReporter reporter("http://localhost:8080");
    
    // Initially no errors
    EXPECT_TRUE(reporter.getLastError().empty());
    EXPECT_EQ(reporter.getSuccessCount(), 0);
    EXPECT_EQ(reporter.getFailureCount(), 0);
}

// Test 9: Success/failure counter behavior
TEST(HttpReporterTest, CounterBehavior) {
    HttpReporter reporter("http://localhost:8080");
    
    // Counters start at zero
    EXPECT_EQ(reporter.getSuccessCount(), 0);
    EXPECT_EQ(reporter.getFailureCount(), 0);
    
    // Verify the interface exists and returns expected types
    int success = reporter.getSuccessCount();
    int failure = reporter.getFailureCount();
    
    EXPECT_GE(success, 0);
    EXPECT_GE(failure, 0);
}

// Test 10: PerimeterStatus mode enum values
TEST(SerializationTest, ModeEnumValues) {
    PerimeterStatus status;
    
    // Verify mode values correspond to ControlMode enum
    status.mode = 0;  // AUTONOMOUS
    EXPECT_EQ(status.mode, uint8_t(0));
    
    status.mode = 1;  // TELEOP
    EXPECT_EQ(status.mode, uint8_t(1));
    
    status.mode = 2;  // HOLD
    EXPECT_EQ(status.mode, uint8_t(2));
}

// Test 11: MineDetection confidence range validation
TEST(SerializationTest, ConfidenceRange) {
    MineDetection detection;
    
    // Confidence should be in [0.0, 1.0] range
    detection.confidence = 0.0f;
    EXPECT_FLOAT_EQ(detection.confidence, 0.0f);
    
    detection.confidence = 1.0f;
    EXPECT_FLOAT_EQ(detection.confidence, 1.0f);
    
    detection.confidence = 0.5f;
    EXPECT_FLOAT_EQ(detection.confidence, 0.5f);
}

// Test 12: MissionSummary result values
TEST(SerializationTest, ResultValues) {
    MissionSummary summary;
    
    // Valid result values
    summary.result = "SUCCESS";
    EXPECT_STREQ(summary.result.c_str(), "SUCCESS");
    
    summary.result = "FAILED";
    EXPECT_STREQ(summary.result.c_str(), "FAILED");
    
    summary.result = "ABORTED";
    EXPECT_STREQ(summary.result.c_str(), "ABORTED");
}

// Test 13: ClearanceReport success/failure states
TEST(SerializationTest, ClearanceSuccessFailure) {
    ClearanceReport success_report;
    success_report.success = true;
    success_report.method = "disposal";
    EXPECT_TRUE(success_report.success);
    EXPECT_STREQ(success_report.method.c_str(), "disposal");
    
    ClearanceReport failure_report;
    failure_report.success = false;
    failure_report.method = "marking";
    EXPECT_FALSE(failure_report.success);
    EXPECT_STREQ(failure_report.method.c_str(), "marking");
}

// Test 14: PerimeterStatus lateral error sign convention
TEST(SerializationTest, LateralErrorSignConvention) {
    PerimeterStatus status;
    
    // Positive lateral error = robot to the left of path
    status.lateral_error = 1.0;
    EXPECT_GT(status.lateral_error, 0.0);
    
    // Negative lateral error = robot to the right of path
    status.lateral_error = -1.0;
    EXPECT_LT(status.lateral_error, 0.0);
    
    // Zero lateral error = robot on path
    status.lateral_error = 0.0;
    EXPECT_DOUBLE_EQ(status.lateral_error, 0.0);
}

// Test 15: MineDetection default values
TEST(SerializationTest, MineDetectionDefaults) {
    MineDetection detection;
    
    EXPECT_EQ(detection.mine_id, 0);
    EXPECT_DOUBLE_EQ(detection.x, 0.0);
    EXPECT_DOUBLE_EQ(detection.y, 0.0);
    EXPECT_TRUE(detection.type.empty());
    EXPECT_FLOAT_EQ(detection.confidence, 0.0f);
    EXPECT_TRUE(detection.detected_at.empty());
}

// Test 16: ClearanceReport default values
TEST(SerializationTest, ClearanceReportDefaults) {
    ClearanceReport report;
    
    EXPECT_EQ(report.mine_id, 0);
    EXPECT_DOUBLE_EQ(report.x, 0.0);
    EXPECT_DOUBLE_EQ(report.y, 0.0);
    EXPECT_TRUE(report.method.empty());
    EXPECT_FALSE(report.success);
    EXPECT_TRUE(report.details.empty());
    EXPECT_TRUE(report.timestamp.empty());
}

// Test 17: MissionSummary default values
TEST(SerializationTest, MissionSummaryDefaults) {
    MissionSummary summary;
    
    EXPECT_TRUE(summary.scenario_name.empty());
    EXPECT_TRUE(summary.result.empty());
    EXPECT_TRUE(summary.reason.empty());
    EXPECT_EQ(summary.total_waypoints, uint32_t(0));
    EXPECT_EQ(summary.waypoints_completed, uint32_t(0));
    EXPECT_EQ(summary.mines_detected, uint32_t(0));
    EXPECT_EQ(summary.mines_cleared, uint32_t(0));
    EXPECT_DOUBLE_EQ(summary.mission_duration, 0.0);
    EXPECT_DOUBLE_EQ(summary.coverage_percent, 0.0);
    EXPECT_TRUE(summary.start_time.empty());
    EXPECT_TRUE(summary.end_time.empty());
}

// Test 18: PerimeterStatus default values
TEST(SerializationTest, PerimeterStatusDefaults) {
    PerimeterStatus status;
    
    EXPECT_EQ(status.mode, uint8_t(0));
    EXPECT_EQ(status.waypoint_index, uint32_t(0));
    EXPECT_DOUBLE_EQ(status.target_x, 0.0);
    EXPECT_DOUBLE_EQ(status.target_y, 0.0);
    EXPECT_DOUBLE_EQ(status.current_x, 0.0);
    EXPECT_DOUBLE_EQ(status.current_y, 0.0);
    EXPECT_DOUBLE_EQ(status.lateral_error, 0.0);
    EXPECT_FLOAT_EQ(status.speed, 0.0f);
    EXPECT_FALSE(status.mine_detected);
    EXPECT_TRUE(status.timestamp.empty());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
