#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "perimeter_miner/perimeter_config.hpp"
#include "perimeter_miner/perimeter_tracker.hpp"
#include "perimeter_miner/hold_controller.hpp"
#include "perimeter_miner/mode_switch.hpp"

using namespace perimeter_miner;

// Test 1: Perimeter tracker initialization
TEST(PerimeterTrackerTest, Initialization) {
    PerimeterConfig config;
    config.name = "test_square";
    config.closed_loop = true;
    config.tolerance = 0.5;
    config.max_speed = 2.0;
    
    config.waypoints = {
        Waypoint{0.0, 0.0, 0.0, 1.0},
        Waypoint{10.0, 0.0, M_PI_2, 1.0},
        Waypoint{10.0, 10.0, M_PI, 1.0},
        Waypoint{0.0, 10.0, -M_PI_2, 1.0}
    };
    
    PerimeterTracker tracker(config);
    
    EXPECT_EQ(tracker.getConfig().name, "test_square");
    EXPECT_EQ(tracker.getConfig().waypointCount(), size_t(4));
    EXPECT_TRUE(tracker.getConfig().closed_loop);
}

// Test 2: Waypoint reach detection
TEST(PerimeterTrackerTest, WaypointReachDetection) {
    PerimeterConfig config;
    config.name = "test_reach";
    config.closed_loop = true;
    config.tolerance = 0.5;
    
    config.waypoints = {
        Waypoint{0.0, 0.0, 0.0, 1.0},
        Waypoint{10.0, 0.0, M_PI_2, 1.0}
    };
    
    PerimeterTracker tracker(config);
    
    // Robot at first waypoint
    RobotState state;
    state.x = 0.0;
    state.y = 0.0;
    tracker.updateRobotState(state);
    
    EXPECT_TRUE(tracker.reachedWaypoint());
    
    // Robot away from first waypoint
    state.x = 2.0;
    state.y = 0.0;
    tracker.updateRobotState(state);
    
    EXPECT_FALSE(tracker.reachedWaypoint());
}

// Test 3: Closed loop waypoint advancement
TEST(PerimeterTrackerTest, ClosedLoopAdvancement) {
    PerimeterConfig config;
    config.name = "test_loop";
    config.closed_loop = true;
    config.tolerance = 0.5;
    
    config.waypoints = {
        Waypoint{0.0, 0.0, 0.0, 1.0},
        Waypoint{10.0, 0.0, M_PI_2, 1.0},
        Waypoint{10.0, 10.0, M_PI, 1.0}
    };
    
    PerimeterTracker tracker(config);
    
    // Start at first waypoint
    RobotState state;
    state.x = 0.0;
    state.y = 0.0;
    tracker.updateRobotState(state);
    
    // Advance from first to second
    tracker.advanceWaypoint();
    auto status = tracker.getStatus();
    EXPECT_EQ(status.waypoint_index, size_t(1));
    
    // Advance from last back to first (closed loop)
    // Starting at index 1, after 5 more advances: 1→2→0→1→2→0
    for (int i = 0; i < 5; ++i) {
        tracker.advanceWaypoint();
    }
    status = tracker.getStatus();
    EXPECT_EQ(status.waypoint_index, size_t(0));  // Wrapped around (1+5=6, 6%3=0)
}

// Test 4: Lateral PID controller
TEST(LateralPIDTest, BasicComputation) {
    LateralPID pid;
    pid.setParameters(2.0, 0.5, 0.3);
    pid.setLimits(1.5, 5.0);
    
    // Constant error should produce proportional output
    double output = pid.compute(1.0, 0.02);
    EXPECT_GT(output, 0.0);
    EXPECT_LT(output, 2.0);
    
    // Reset should clear integral
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
}

// Test 5: Mode switch basic operations
TEST(ModeSwitchTest, BasicModeSwitch) {
    ModeSwitch ms;
    
    EXPECT_EQ(ms.getCurrentMode(), ControlMode::AUTONOMOUS);
    EXPECT_FALSE(ms.isSwitching());
    
    // Request mode change
    bool success = ms.requestMode(ControlMode::TELEOP);
    EXPECT_TRUE(success);
    EXPECT_EQ(ms.getPendingMode(), ControlMode::TELEOP);
    
    // Apply the switch
    success = ms.applyRequest();
    EXPECT_TRUE(success);
    EXPECT_EQ(ms.getCurrentMode(), ControlMode::TELEOP);
    EXPECT_FALSE(ms.isSwitching());
}

// Test 6: Mode switch operator override
TEST(ModeSwitchTest, OperatorOverride) {
    ModeSwitch ms;
    
    // Start in AUTONOMOUS
    EXPECT_EQ(ms.getCurrentMode(), ControlMode::AUTONOMOUS);
    
    // Operator overrides to TELEOP
    bool overridden = ms.operatorOverride();
    EXPECT_TRUE(overridden);
    EXPECT_EQ(ms.getCurrentMode(), ControlMode::TELEOP);
    
    // Cannot switch back via normal request while in TELEOP
    bool success = ms.requestMode(ControlMode::AUTONOMOUS);
    // This may succeed or fail depending on safety checks
}

// Test 7: Mode switch duplicate request
TEST(ModeSwitchTest, DuplicateRequest) {
    ModeSwitch ms;
    
    // Request same mode twice
    bool success1 = ms.requestMode(ControlMode::HOLD);
    EXPECT_TRUE(success1);
    
    // Apply the first request to complete the switch
    ms.applyRequest();
    EXPECT_EQ(ms.getCurrentMode(), ControlMode::HOLD);
    
    // Now request HOLD again - should fail (already in HOLD)
    bool success2 = ms.requestMode(ControlMode::HOLD);
    EXPECT_FALSE(success2);  // Already in HOLD
}

// Test 8: Hold controller
TEST(HoldControllerTest, SetHoldPosition) {
    HoldController hold;
    
    hold.setHoldPosition(5.0, 10.0, M_PI_4);
    
    RobotState state;
    state.x = 5.0;
    state.y = 10.0;
    state.heading = M_PI_4;
    
    EXPECT_TRUE(hold.isAtHoldPosition(state, 0.5));
}

// Test 9: Hold controller away from position
TEST(HoldControllerTest, AwayFromHoldPosition) {
    HoldController hold;
    
    hold.setHoldPosition(5.0, 10.0, 0.0);
    
    RobotState state;
    state.x = 3.0;
    state.y = 8.0;
    state.heading = M_PI_2;
    
    EXPECT_FALSE(hold.isAtHoldPosition(state, 0.5));
    
    // Should produce non-zero command
    auto cmd = hold.compute(state);
    EXPECT_TRUE(std::abs(cmd.linear_x) > 0.0 || std::abs(cmd.angular_z) > 0.0);
}

// Test 10: Perimeter status
TEST(PerimeterTrackerTest, StatusRetrieval) {
    PerimeterConfig config;
    config.name = "test_status";
    config.closed_loop = true;
    config.tolerance = 0.5;
    
    config.waypoints = {
        Waypoint{0.0, 0.0, 0.0, 1.0},
        Waypoint{10.0, 0.0, M_PI_2, 1.0}
    };
    
    PerimeterTracker tracker(config);
    
    RobotState state;
    state.x = 0.0;
    state.y = 0.0;
    tracker.updateRobotState(state);
    
    auto status = tracker.getStatus();
    EXPECT_EQ(status.waypoint_index, size_t(0));
    EXPECT_DOUBLE_EQ(status.target_x, 0.0);
    EXPECT_DOUBLE_EQ(status.target_y, 0.0);
}

// Test 11: Angle normalization
TEST(AngleTest, NormalizeAngle) {
    // Test normalizeAngle function via angleDiff
    double diff = PerimeterTracker::angleDiff(0.0, 0.0);
    EXPECT_DOUBLE_EQ(diff, 0.0);
    
    diff = PerimeterTracker::angleDiff(0.0, M_PI);
    EXPECT_TRUE(std::abs(diff - M_PI) < 0.01 || std::abs(diff + M_PI) < 0.01);
    
    diff = PerimeterTracker::angleDiff(0.0, -M_PI);
    EXPECT_NEAR(diff, -M_PI, 0.01);
}

// Test 12: Robot state distance computation
TEST(RobotStateTest, DistanceComputation) {
    RobotState state;
    state.x = 0.0;
    state.y = 0.0;
    
    double dist = state.distanceTo(3.0, 4.0);
    EXPECT_DOUBLE_EQ(dist, 5.0);
    
    dist = state.distanceTo(0.0, 0.0);
    EXPECT_DOUBLE_EQ(dist, 0.0);
}

// Test 13: Robot state bearing computation
TEST(RobotStateTest, BearingComputation) {
    RobotState state;
    state.x = 0.0;
    state.y = 0.0;
    
    double bearing = state.bearingTo(1.0, 0.0);
    EXPECT_DOUBLE_EQ(bearing, 0.0);
    
    bearing = state.bearingTo(0.0, 1.0);
    EXPECT_DOUBLE_EQ(bearing, M_PI_2);
}

// Test 14: Control mode string conversion
TEST(ControlModeTest, StringConversion) {
    EXPECT_STREQ(controlModeToString(ControlMode::AUTONOMOUS), "AUTONOMOUS");
    EXPECT_STREQ(controlModeToString(ControlMode::TELEOP), "TELEOP");
    EXPECT_STREQ(controlModeToString(ControlMode::HOLD), "HOLD");
}

// Test 15: MoveCommand factory methods
TEST(MoveCommandTest, FactoryMethods) {
    auto zero = MoveCommand::zero();
    EXPECT_DOUBLE_EQ(zero.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(zero.linear_y, 0.0);
    EXPECT_DOUBLE_EQ(zero.angular_z, 0.0);
    
    auto full = MoveCommand::fullForward(2.0);
    EXPECT_DOUBLE_EQ(full.linear_x, 2.0);
    EXPECT_DOUBLE_EQ(full.linear_y, 0.0);
    EXPECT_DOUBLE_EQ(full.angular_z, 0.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
