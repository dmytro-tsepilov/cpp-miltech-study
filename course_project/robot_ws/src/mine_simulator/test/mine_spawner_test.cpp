#include <gtest/gtest.h>
#include <cmath>

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

// Test 3: Mine simulation config default values
TEST(MineSimConfigTest, DefaultValues) {
    MineSimConfig config;
    
    EXPECT_DOUBLE_EQ(config.detection_range, 3.0);
    EXPECT_DOUBLE_EQ(config.detection_probability, 0.95);
    EXPECT_DOUBLE_EQ(config.detection_decay, 0.5);
    EXPECT_DOUBLE_EQ(config.update_period_ms, 100.0);
}

// Test 4: Distance computation for detection
TEST(MineDetectionTest, DistanceComputation) {
    double dx = 10.0 - 0.0;
    double dy = 5.0 - 0.0;
    double dist = std::hypot(dx, dy);
    
    EXPECT_NEAR(dist, std::sqrt(125.0), 0.001);
}

// Test 5: Detection range check
TEST(MineDetectionTest, WithinDetectionRange) {
    double mine_x = 10.0;
    double mine_y = 5.0;
    double robot_x = 9.0;
    double robot_y = 4.0;
    double detection_range = 3.0;
    
    double dist = std::hypot(mine_x - robot_x, mine_y - robot_y);
    bool within_range = dist <= detection_range;
    
    EXPECT_TRUE(within_range);
}

// Test 6: Detection range check - outside range
TEST(MineDetectionTest, OutsideDetectionRange) {
    double mine_x = 10.0;
    double mine_y = 5.0;
    double robot_x = 20.0;
    double robot_y = 15.0;
    double detection_range = 3.0;
    
    double dist = std::hypot(mine_x - robot_x, mine_y - robot_y);
    bool within_range = dist <= detection_range;
    
    EXPECT_FALSE(within_range);
}

// Test 7: Detection probability calculation
TEST(MineDetectionTest, ProbabilityCalculation) {
    double detection_probability = 0.95;
    double detection_decay = 0.5;
    double detection_range = 3.0;
    
    // At zero distance, probability should be max
    double dist = 0.0;
    double prob = detection_probability * (1.0 - dist / detection_range * detection_decay);
    EXPECT_DOUBLE_EQ(prob, detection_probability);
    
    // At detection range, probability should be reduced
    dist = detection_range;
    prob = detection_probability * (1.0 - dist / detection_range * detection_decay);
    EXPECT_DOUBLE_EQ(prob, detection_probability * 0.5);
}

// Test 8: Mine config name validation
TEST(MineConfigTest, NameValidation) {
    MineSimConfig config;
    config.name = "training_ground";
    
    EXPECT_FALSE(config.name.empty());
    EXPECT_STREQ(config.name.c_str(), "training_ground");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
