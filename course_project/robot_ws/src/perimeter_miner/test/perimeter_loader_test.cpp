// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Test suite for PerimeterLoader - YAML configuration parsing.

#include <gtest/gtest.h>
#include <fstream>
#include <sstream>

#include "perimeter_miner/perimeter_config.hpp"
#include "perimeter_miner/perimeter_loader.hpp"

using perimeter_miner::PerimeterConfig;
using perimeter_miner::PerimeterLoader;
using perimeter_miner::Waypoint;

// Helper to create a temporary file with content
static std::string createTempFile(const std::string& content)
{
  // Create temp file in current directory
  std::ofstream tmp("/tmp/test_perimeter_config.yaml");
  tmp << content;
  tmp.close();
  return "/tmp/test_perimeter_config.yaml";
}

// Test 1: Load valid training ground config
TEST(PerimeterLoaderTest, LoadValidConfig)
{
  std::string yaml = R"(
name: training_ground
closed_loop: true
tolerance: 0.5
max_speed: 2.0
min_turn_radius: 1.0

waypoints:
  - {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
  - {x: 20.0, y: 0.0, heading: 1.5708, radius: 1.0}
  - {x: 20.0, y: 20.0, heading: 3.1416, radius: 1.0}
  - {x: 0.0, y: 20.0, heading: -1.5708, radius: 1.0}

mines:
  - {id: 1, x: 10.0, y: 5.0, type: "anti-tank"}
)";

  auto path = createTempFile(yaml);
  PerimeterConfig config = PerimeterLoader::loadFromFile(path);

  EXPECT_EQ(config.name, "training_ground");
  EXPECT_TRUE(config.closed_loop);
  EXPECT_DOUBLE_EQ(config.tolerance, 0.5);
  EXPECT_DOUBLE_EQ(config.max_speed, 2.0);
  EXPECT_DOUBLE_EQ(config.min_turn_radius, 1.0);
  EXPECT_EQ(config.waypointCount(), size_t(4));
}

// Test 2: Load open perimeter config
TEST(PerimeterLoaderTest, LoadOpenPerimeter)
{
  std::string yaml = R"(
name: patrol_alpha
closed_loop: false
tolerance: 0.5
max_speed: 1.5

waypoints:
  - {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
  - {x: 15.0, y: 5.0, heading: 0.3, radius: 1.0}
  - {x: 30.0, y: 10.0, heading: 0.33, radius: 1.0}
)";

  auto path = createTempFile(yaml);
  PerimeterConfig config = PerimeterLoader::loadFromFile(path);

  EXPECT_EQ(config.name, "patrol_alpha");
  EXPECT_FALSE(config.closed_loop);
  EXPECT_DOUBLE_EQ(config.max_speed, 1.5);
  EXPECT_EQ(config.waypointCount(), size_t(3));
}

// Test 3: Load config with defaults when values missing
TEST(PerimeterLoaderTest, LoadWithDefaults)
{
  std::string yaml = R"(
name: minimal_config
waypoints:
  - {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
)";

  auto path = createTempFile(yaml);
  PerimeterConfig config = PerimeterLoader::loadFromFile(path);

  EXPECT_EQ(config.name, "minimal_config");
  // Defaults should be applied
  EXPECT_TRUE(config.closed_loop);  // default true
  EXPECT_DOUBLE_EQ(config.tolerance, 0.5);  // default 0.5
  EXPECT_DOUBLE_EQ(config.max_speed, 2.0);  // default 2.0
}

// Test 4: Load large patrol config (16 waypoints)
TEST(PerimeterLoaderTest, LoadLargePatrol)
{
  std::string yaml = R"(
name: large_patrol
closed_loop: true
tolerance: 1.0
max_speed: 2.5
min_turn_radius: 1.5

waypoints:
  - {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
  - {x: 10.0, y: 0.0, heading: 0.314, radius: 1.0}
  - {x: 20.0, y: 2.0, heading: 0.628, radius: 1.0}
  - {x: 28.0, y: 6.0, heading: 0.942, radius: 1.0}
  - {x: 32.0, y: 12.0, heading: 1.257, radius: 1.0}
  - {x: 30.0, y: 20.0, heading: 1.571, radius: 1.0}
  - {x: 25.0, y: 26.0, heading: 1.885, radius: 1.0}
  - {x: 18.0, y: 28.0, heading: 2.199, radius: 1.0}
  - {x: 10.0, y: 26.0, heading: 2.513, radius: 1.0}
  - {x: 4.0, y: 20.0, heading: 2.827, radius: 1.0}
  - {x: 2.0, y: 12.0, heading: 3.142, radius: 1.0}
  - {x: 4.0, y: 6.0, heading: -2.827, radius: 1.0}
  - {x: 10.0, y: 2.0, heading: -2.513, radius: 1.0}
  - {x: 16.0, y: 0.5, heading: -2.199, radius: 1.0}
  - {x: 22.0, y: 1.0, heading: -1.885, radius: 1.0}
  - {x: 26.0, y: 5.0, heading: -1.571, radius: 1.0}
)";

  auto path = createTempFile(yaml);
  PerimeterConfig config = PerimeterLoader::loadFromFile(path);

  EXPECT_EQ(config.name, "large_patrol");
  EXPECT_EQ(config.waypointCount(), size_t(16));
  EXPECT_DOUBLE_EQ(config.max_speed, 2.5);
}

// Test 5: Load from non-existent file returns empty config
TEST(PerimeterLoaderTest, LoadNonExistentFile)
{
  PerimeterConfig config = PerimeterLoader::loadFromFile("/nonexistent/path.yaml");

  EXPECT_TRUE(config.waypoints.empty());
  // Name is empty when file fails to load (default-constructed PerimeterConfig)
  EXPECT_TRUE(config.name.empty());
}

// Test 6: LoadFromString with minimal YAML
TEST(PerimeterLoaderTest, LoadFromStringMinimal)
{
  std::string yaml = R"(
name: string_test
waypoints:
  - {x: 5.0, y: 10.0, heading: 0.5, radius: 2.0}
)";

  PerimeterConfig config = PerimeterLoader::loadFromString(yaml);

  EXPECT_EQ(config.name, "string_test");
  EXPECT_EQ(config.waypointCount(), size_t(1));
  EXPECT_DOUBLE_EQ(config.waypoints[0].x, 5.0);
  EXPECT_DOUBLE_EQ(config.waypoints[0].y, 10.0);
}

// Test 7: LoadFromString with comments
TEST(PerimeterLoaderTest, LoadWithComments)
{
  std::string yaml = R"(
# This is a comment
name: commented_config

# Another comment
closed_loop: true
tolerance: 0.8

waypoints:
  # First waypoint
  - {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
  # Second waypoint
  - {x: 15.0, y: 15.0, heading: 1.0, radius: 1.5}
)";

  PerimeterConfig config = PerimeterLoader::loadFromString(yaml);

  EXPECT_EQ(config.name, "commented_config");
  EXPECT_TRUE(config.closed_loop);
  EXPECT_DOUBLE_EQ(config.tolerance, 0.8);
  EXPECT_EQ(config.waypointCount(), size_t(2));
}

// Test 8: Load config with whitespace variations
TEST(PerimeterLoaderTest, LoadWithWhitespace)
{
  std::string yaml = R"(
name:    whitespace_test
closed_loop:   true
tolerance:   0.6
max_speed :   1.8

waypoints:
    - {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
      - {x: 10.0, y: 10.0, heading: 0.5, radius: 1.0}
)";

  PerimeterConfig config = PerimeterLoader::loadFromString(yaml);

  EXPECT_EQ(config.name, "whitespace_test");
  EXPECT_TRUE(config.closed_loop);
  EXPECT_EQ(config.waypointCount(), size_t(2));
}

// Test 9: GetWaypoint with closed loop wrapping
TEST(PerimeterLoaderTest, WaypointWrapping)
{
  std::string yaml = R"(
name: wrap_test
closed_loop: true

waypoints:
  - {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
  - {x: 10.0, y: 0.0, heading: 0.5, radius: 1.0}
  - {x: 10.0, y: 10.0, heading: 1.0, radius: 1.0}
)";

  PerimeterConfig config = PerimeterLoader::loadFromString(yaml);

  // Access beyond bounds should wrap for closed loop
  const auto& wp0 = config.getWaypoint(0);
  EXPECT_DOUBLE_EQ(wp0.x, 0.0);

  const auto& wp2 = config.getWaypoint(2);
  EXPECT_DOUBLE_EQ(wp2.x, 10.0);

  // Index 3 should wrap to 0 for closed loop with 3 waypoints
  const auto& wp3 = config.getWaypoint(3);
  EXPECT_DOUBLE_EQ(wp3.x, 0.0);  // Wrapped
}

// Test 10: Last error message
TEST(PerimeterLoaderTest, GetLastError)
{
  // Load non-existent file
  PerimeterLoader::loadFromFile("/nonexistent/file.yaml");

  std::string last_error = PerimeterLoader::getLastError();
  EXPECT_FALSE(last_error.empty());
}

// Test 11: Empty waypoints section
TEST(PerimeterLoaderTest, EmptyWaypoints)
{
  std::string yaml = R"(
name: no_waypoints
closed_loop: true
waypoints:
)";

  PerimeterConfig config = PerimeterLoader::loadFromString(yaml);

  EXPECT_TRUE(config.waypoints.empty());
}

// Test 12: Waypoint with only required fields
TEST(PerimeterLoaderTest, MinimalWaypoint)
{
  std::string yaml = R"(
name: minimal_wp
waypoints:
  - {x: 5.0, y: 10.0}
)";

  PerimeterConfig config = PerimeterLoader::loadFromString(yaml);

  EXPECT_EQ(config.waypointCount(), size_t(1));
  // Default heading and radius should be applied
  EXPECT_DOUBLE_EQ(config.waypoints[0].x, 5.0);
  EXPECT_DOUBLE_EQ(config.waypoints[0].y, 10.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
