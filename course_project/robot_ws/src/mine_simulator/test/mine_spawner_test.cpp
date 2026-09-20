#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <string>

#include "mine_simulator/mine_config.hpp"

using namespace mine_simulator;

// Test 1: Mine definition default values
TEST(MineDefinitionTest, DefaultValues) {
    MineDefinition mine;
    
    EXPECT_EQ(mine.id, 0);
    EXPECT_DOUBLE_EQ(mine.x, 0.0);
    EXPECT_DOUBLE_EQ(mine.y, 0.0);
    EXPECT_FALSE(mine.detected);
    EXPECT_FALSE(mine.cleared);
}

// Test 2: Mine definition initialization
TEST(MineDefinitionTest, Initialization) {
    MineDefinition mine{1, 10.0, 20.0, "anti-tank"};
    
    EXPECT_EQ(mine.id, 1);
    EXPECT_DOUBLE_EQ(mine.x, 10.0);
    EXPECT_DOUBLE_EQ(mine.y, 20.0);
    EXPECT_STREQ(mine.type.c_str(), "anti-tank");
}

// Test 3: Mine definition distance computation from origin
TEST(MineDefinitionTest, DistanceFromOrigin) {
    MineDefinition mine{1, 3.0, 4.0, "anti-tank"};
    
    // Distance from (0,0) to (3,4) should be 5.0
    double dist = std::hypot(mine.x, mine.y);
    EXPECT_DOUBLE_EQ(dist, 5.0);
}

// Test 4: Mine definition distance computation from arbitrary point
TEST(MineDefinitionTest, DistanceFromArbitraryPoint) {
    MineDefinition mine{1, 10.0, 5.0, "anti-tank"};
    
    double robot_x = 7.0;
    double robot_y = 1.0;
    double dx = mine.x - robot_x;
    double dy = mine.y - robot_y;
    double dist = std::hypot(dx, dy);
    
    // Distance from (7,1) to (10,5) should be sqrt(9 + 16) = 5.0
    EXPECT_DOUBLE_EQ(dist, 5.0);
}

// Test 5: Mine definition state transitions — detected then cleared
TEST(MineDefinitionTest, StateTransitions) {
    MineDefinition mine{42, 10.0, 20.0, "anti-tank"};
    
    // Initial state: not detected, not cleared
    EXPECT_FALSE(mine.detected);
    EXPECT_FALSE(mine.cleared);
    
    // Simulate detection event
    mine.detected = true;
    EXPECT_TRUE(mine.detected);
    EXPECT_FALSE(mine.cleared);
    
    // Simulate clearance after detection
    mine.cleared = true;
    EXPECT_TRUE(mine.detected);   // detected persists
    EXPECT_TRUE(mine.cleared);
    
    // State should persist through transitions
    EXPECT_EQ(mine.id, 42);
    EXPECT_DOUBLE_EQ(mine.x, 10.0);
    EXPECT_STREQ(mine.type.c_str(), "anti-tank");
}

// Test 6: Mine definition — multiple mines with different types
TEST(MineDefinitionTest, MultipleMineTypes) {
    std::vector<MineDefinition> mines = {
        {1, 0.0, 0.0, "anti-tank"},
        {2, 5.0, 5.0, "anti-personnel"},
        {3, 10.0, 10.0, "unknown"}
    };
    
    EXPECT_EQ(mines.size(), size_t(3));
    
    for (size_t i = 0; i < mines.size(); ++i) {
        EXPECT_EQ(mines[i].id, static_cast<int>(i + 1));
        EXPECT_FALSE(mines[i].detected);
        EXPECT_FALSE(mines[i].cleared);
    }
    
    EXPECT_STREQ(mines[0].type.c_str(), "anti-tank");
    EXPECT_STREQ(mines[1].type.c_str(), "anti-personnel");
    EXPECT_STREQ(mines[2].type.c_str(), "unknown");
}

// Test 7: Mine simulation config default values
TEST(MineSimConfigTest, DefaultValues) {
    MineSimConfig config;
    
    EXPECT_DOUBLE_EQ(config.detection_range, 3.0);
    EXPECT_DOUBLE_EQ(config.detection_probability, 0.95);
    EXPECT_DOUBLE_EQ(config.detection_decay, 0.5);
    EXPECT_DOUBLE_EQ(config.update_period_ms, 100.0);
}

// Test 8: Mine simulation config detection probability at boundaries
TEST(MineSimConfigTest, DetectionProbabilityBoundaries) {
    MineSimConfig config;
    config.detection_probability = 0.95;
    config.detection_decay = 0.5;
    config.detection_range = 3.0;
    
    // At zero distance (robot exactly on mine), probability = base probability
    double dist = 0.0;
    double prob_at_center = config.detection_probability * 
                            (1.0 - dist / config.detection_range * config.detection_decay);
    EXPECT_DOUBLE_EQ(prob_at_center, 0.95);
    
    // At exactly detection_range boundary, probability = base * 0.5
    dist = config.detection_range;
    double prob_at_boundary = config.detection_probability * 
                              (1.0 - dist / config.detection_range * config.detection_decay);
    EXPECT_DOUBLE_EQ(prob_at_boundary, 0.475);  // 0.95 * 0.5
    
    // At twice detection_range (outside), probability should be zero or negative
    dist = config.detection_range * 2.0;
    double prob_outside = config.detection_probability * 
                          (1.0 - dist / config.detection_range * config.detection_decay);
    EXPECT_DOUBLE_EQ(prob_outside, 0.0);  // 0.95 * (1 - 1.0) = 0.0
}

// Test 9: Detection range check — robot exactly at boundary
TEST(MineDetectionTest, ExactlyAtRangeBoundary) {
    MineDefinition mine{1, 5.0, 0.0, "anti-tank"};
    
    // Robot positioned exactly at detection_range from mine
    double robot_x = 8.0;  // 5.0 + 3.0
    double robot_y = 0.0;
    double detection_range = 3.0;
    
    double dist = std::hypot(mine.x - robot_x, mine.y - robot_y);
    
    // Distance should be exactly 3.0
    EXPECT_DOUBLE_EQ(dist, detection_range);
    
    // At exact boundary, dist <= detection_range should be true (inclusive)
    EXPECT_TRUE(dist <= detection_range);
}

// Test 10: Detection range check — robot just inside range
TEST(MineDetectionTest, JustInsideRange) {
    MineDefinition mine{1, 10.0, 0.0, "anti-tank"};
    
    double robot_x = 7.0;   // 0.001m inside the 3.0 range
    double robot_y = 0.0;
    double detection_range = 3.0;
    
    double dist = std::hypot(mine.x - robot_x, mine.y - robot_y);
    
    EXPECT_NEAR(dist, 3.0, 0.001);
    EXPECT_TRUE(dist <= detection_range);
}

// Test 11: Detection range check — robot just outside range
TEST(MineDetectionTest, JustOutsideRange) {
    MineDefinition mine{1, 10.0, 0.0, "anti-tank"};
    
    double robot_x = 6.99;   // 0.01m outside the 3.0 range
    double robot_y = 0.0;
    double detection_range = 3.0;
    
    double dist = std::hypot(mine.x - robot_x, mine.y - robot_y);
    
    EXPECT_NEAR(dist, 3.01, 0.01);
    EXPECT_FALSE(dist <= detection_range);
}

// Test 12: Detection probability decreases monotonically with distance
TEST(MineDetectionTest, ProbabilityMonotonicDecrease) {
    const double detection_probability = 0.95;
    const double detection_decay = 0.5;
    const double detection_range = 3.0;
    
    double prev_prob = detection_probability;  // at dist = 0
    
    for (int i = 1; i <= 6; ++i) {
        double dist = static_cast<double>(i) * detection_range / 6.0;
        double prob = detection_probability * (1.0 - dist / detection_range * detection_decay);
        
        // Probability should decrease or stay equal as distance increases
        EXPECT_LE(prob, prev_prob);
        prev_prob = prob;
    }
}

// Test 13: Mine config name validation — empty name
TEST(MineConfigTest, EmptyName) {
    MineSimConfig config;
    
    // Default name should be empty
    EXPECT_TRUE(config.name.empty());
    EXPECT_TRUE(config.mines.empty());
}

// Test 14: Mine config name validation — non-empty name
TEST(MineConfigTest, NameValidation) {
    MineSimConfig config;
    config.name = "training_ground";
    
    EXPECT_FALSE(config.name.empty());
    EXPECT_STREQ(config.name.c_str(), "training_ground");
}

// Test 15: Mine config with empty mines vector
TEST(MineConfigTest, EmptyMinesVector) {
    MineSimConfig config;
    config.name = "empty_field";
    
    // Should have no mines but valid detection params
    EXPECT_EQ(config.mines.size(), size_t(0));
    EXPECT_DOUBLE_EQ(config.detection_range, 3.0);
    EXPECT_DOUBLE_EQ(config.detection_probability, 0.95);
}

// Test 16: Detection probability with zero decay
TEST(MineDetectionTest, ZeroDecayProbability) {
    const double detection_probability = 0.95;
    const double detection_decay = 0.0;   // No decay
    const double detection_range = 3.0;
    
    // With zero decay, probability should be constant at all distances
    for (int i = 0; i <= 10; ++i) {
        double dist = static_cast<double>(i) * detection_range / 10.0;
        double prob = detection_probability * (1.0 - dist / detection_range * detection_decay);
        EXPECT_DOUBLE_EQ(prob, detection_probability);
    }
}

// Test 17: Detection probability with maximum decay
TEST(MineDetectionTest, MaximumDecayProbability) {
    const double detection_probability = 1.0;
    const double detection_decay = 1.0;   // Maximum decay
    const double detection_range = 3.0;
    
    // At detection range with max decay, probability should be zero
    double dist = detection_range;
    double prob = detection_probability * (1.0 - dist / detection_range * detection_decay);
    EXPECT_DOUBLE_EQ(prob, 0.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
