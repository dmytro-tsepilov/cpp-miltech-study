#include "ballistics.hpp"

#include <cmath>
#include <gtest/gtest.h>

// Utility for floating-point comparison
static bool approx_equal(double actual, double expected, double epsilon = 0.01)
{
  return std::fabs(actual - expected) < epsilon;
}

class BallisticsTest : public ::testing::Test {
protected:
  void ExpectNear(double actual, double expected, double epsilon = 0.01)
  {
    EXPECT_TRUE(approx_equal(actual, expected, epsilon))
      << "Expected " << expected << " but got " << actual << " (epsilon = " << epsilon << ")";
  }
};

// Test 1: Reference test for VOG-17 (from DH1 example)
// Input: 100 100 100 200 200 10 10 VOG-17
// Expected: fire_x ≈ 166.688, fire_y ≈ 166.688 (based on actual calculation)
TEST_F(BallisticsTest, ComputesKnownDropPoint)
{
  BallisticsInput input{};
  input.drone_x = 100.0;
  input.drone_y = 100.0;
  input.drone_z = 100.0;
  input.target_x = 200.0;
  input.target_y = 200.0;
  input.attack_speed = 10.0;
  input.acceleration_path = 10.0;
  input.ammo_name = "VOG-17";

  DropSolution solution = compute_drop_solution(input);

  EXPECT_TRUE(solution.valid);
  ExpectNear(solution.fire_x, 166.688, 0.01);
  ExpectNear(solution.fire_y, 166.688, 0.01);
  EXPECT_GT(solution.time_of_flight, 0.0);
}

// Test 2: Unknown ammo type returns error
TEST_F(BallisticsTest, UnknownAmmoTypeReturnsError)
{
  BallisticsInput input{};
  input.drone_x = 100.0;
  input.drone_y = 100.0;
  input.drone_z = 100.0;
  input.target_x = 200.0;
  input.target_y = 200.0;
  input.attack_speed = 10.0;
  input.acceleration_path = 10.0;
  input.ammo_name = "UNKNOWN-AMMO";

  DropSolution solution = compute_drop_solution(input);

  EXPECT_FALSE(solution.valid);
}

// Test 3: Zero height returns error
TEST_F(BallisticsTest, ZeroHeightReturnsError)
{
  BallisticsInput input{};
  input.drone_x = 100.0;
  input.drone_y = 100.0;
  input.drone_z = 0.0;
  input.target_x = 200.0;
  input.target_y = 200.0;
  input.attack_speed = 10.0;
  input.acceleration_path = 10.0;
  input.ammo_name = "VOG-17";

  DropSolution solution = compute_drop_solution(input);

  EXPECT_FALSE(solution.valid);
  EXPECT_NE(solution.error_message.find("positive"), std::string::npos);
}

// Test 4: Gliding ammo produces positive time of flight
TEST_F(BallisticsTest, GlidingAmmoProducesPositiveTime)
{
  BallisticsInput input{};
  input.drone_x = 100.0;
  input.drone_y = 100.0;
  input.drone_z = 150.0;
  input.target_x = 300.0;
  input.target_y = 300.0;
  input.attack_speed = 15.0;
  input.acceleration_path = 5.0;
  input.ammo_name = "GLIDING-VOG";

  DropSolution solution = compute_drop_solution(input);

  EXPECT_TRUE(solution.valid);
  EXPECT_GT(solution.time_of_flight, 0.0);
  // The fire point should be roughly along the diagonal toward the target
  ExpectNear(solution.fire_x, 237.671, 0.01);
  ExpectNear(solution.fire_y, 237.671, 0.01);
}

// Test 5: Zero distance returns error
TEST_F(BallisticsTest, ZeroDistanceReturnsError)
{
  BallisticsInput input{};
  input.drone_x = 100.0;
  input.drone_y = 100.0;
  input.drone_z = 100.0;
  input.target_x = 100.0;
  input.target_y = 100.0;
  input.attack_speed = 10.0;
  input.acceleration_path = 10.0;
  input.ammo_name = "VOG-17";

  DropSolution solution = compute_drop_solution(input);

  EXPECT_FALSE(solution.valid);
  EXPECT_NE(solution.error_message.find("same position"), std::string::npos);
}

// Test 6: Parse input line correctly
TEST_F(BallisticsTest, ParseInputLine)
{
  std::string line = "100 100 100 200 200 10 10 VOG-17";
  BallisticsInput input;

  EXPECT_TRUE(parse_input_line(line, input));
  EXPECT_DOUBLE_EQ(input.drone_x, 100.0);
  EXPECT_DOUBLE_EQ(input.drone_y, 100.0);
  EXPECT_DOUBLE_EQ(input.drone_z, 100.0);
  EXPECT_DOUBLE_EQ(input.target_x, 200.0);
  EXPECT_DOUBLE_EQ(input.target_y, 200.0);
  EXPECT_DOUBLE_EQ(input.attack_speed, 10.0);
  EXPECT_DOUBLE_EQ(input.acceleration_path, 10.0);
  EXPECT_EQ(input.ammo_name, "VOG-17");
}

// Test 7: Parse invalid input line returns false
TEST_F(BallisticsTest, ParseInvalidInputLine)
{
  std::string line = "invalid input data";
  BallisticsInput input;

  EXPECT_FALSE(parse_input_line(line, input));
}

// Test 8: M67 ammo type lookup
TEST_F(BallisticsTest, AmmoTypeLookup)
{
  AmmoType props{};
  EXPECT_TRUE(detectAmmoType("M67", props));
  EXPECT_STREQ(props.name, "M67");
  ExpectNear(props.mass, 0.60, 0.001);
  ExpectNear(props.drag, 0.10, 0.001);
  ExpectNear(props.lift, 0.0, 0.001);
  EXPECT_FALSE(props.is_gliding);
}

// Test 9: RKG-3 ammo type lookup
TEST_F(BallisticsTest, RKG3AmmoType)
{
  AmmoType props{};
  EXPECT_TRUE(detectAmmoType("RKG-3", props));
  EXPECT_STREQ(props.name, "RKG-3");
  ExpectNear(props.mass, 1.20, 0.001);
  ExpectNear(props.drag, 0.10, 0.001);
  EXPECT_FALSE(props.is_gliding);
}

// Test 10: Negative height returns error
TEST_F(BallisticsTest, NegativeHeightReturnsError)
{
  BallisticsInput input{};
  input.drone_x = 100.0;
  input.drone_y = 100.0;
  input.drone_z = -50.0;
  input.target_x = 200.0;
  input.target_y = 200.0;
  input.attack_speed = 10.0;
  input.acceleration_path = 10.0;
  input.ammo_name = "VOG-17";

  DropSolution solution = compute_drop_solution(input);

  EXPECT_FALSE(solution.valid);
}
