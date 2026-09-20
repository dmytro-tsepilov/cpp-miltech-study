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

#include "perimeter_miner/perimeter_loader.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace perimeter_miner
{

// Define static member
// NOLINTNEXTLINE(runtime/string)
std::string PerimeterLoader::last_error_ = "";


// Helper: trim whitespace
static std::string trim(const std::string& str)
{
  size_t first = str.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  size_t last = str.find_last_not_of(" \t\r\n");
  return str.substr(first, last - first + 1);
}

// Helper: check if line is a comment or empty
static bool isCommentOrEmpty(const std::string& line)
{
  std::string trimmed = trim(line);
  return trimmed.empty() || trimmed[0] == '#';
}

size_t PerimeterLoader::findKey(
  const std::string& yaml, const std::string& key, size_t start)
{
  // Look for "key:" pattern in yaml starting from 'start'
  std::string pattern = key + ":";
  size_t pos = yaml.find(pattern, start);

  while (pos != std::string::npos) {
    // Check character after the key - must be space, tab, or newline (YAML key delimiter)
    size_t end_pos = pos + pattern.size();
    bool valid_after = false;
    if (end_pos >= yaml.size()) {
      valid_after = true;
    } else if (yaml[end_pos] == ' ' || yaml[end_pos] == '\t' ||
               yaml[end_pos] == '\n' || yaml[end_pos] == '\r') {
      valid_after = true;
    }

    // Check character before the key - must be start of string, newline, or whitespace (line start)
    bool valid_before = false;
    if (pos == 0) {
      valid_before = true;
    } else if (yaml[pos - 1] == '\n' || yaml[pos - 1] == '\r' ||
               yaml[pos - 1] == ' ' || yaml[pos - 1] == '\t') {
      valid_before = true;
    }

    if (valid_before && valid_after) {
      return pos;
    }
    pos = yaml.find(pattern, pos + 1);
  }
  return std::string::npos;
}

bool PerimeterLoader::extractValue(
  const std::string& yaml, const std::string& key, std::string& out_value)
{
  size_t pos = findKey(yaml, key);
  if (pos == std::string::npos) {
    return false;
  }

  // Get the value after "key:"
  size_t val_start = pos + key.size() + 1;  // skip "key:"
  if (val_start >= yaml.size()) {
    out_value = "";
    return true;
  }

  // Find end of line
  size_t line_end = yaml.find('\n', val_start);
  if (line_end == std::string::npos) {
    out_value = trim(yaml.substr(val_start));
  } else {
    out_value = trim(yaml.substr(val_start, line_end - val_start));
  }
  return true;
}

bool PerimeterLoader::extractDouble(
  const std::string& yaml, const std::string& key, double& out_value)
{
  std::string str_val;
  if (!extractValue(yaml, key, str_val)) {
    return false;
  }

  try {
    out_value = std::stod(str_val);
    return true;
  } catch (...) {
    last_error_ = "Failed to parse double for key: " + key;
    return false;
  }
}

bool PerimeterLoader::extractBool(
  const std::string& yaml, const std::string& key, bool& out_value)
{
  std::string str_val;
  if (!extractValue(yaml, key, str_val)) {
    return false;
  }

  if (str_val == "true" || str_val == "True" || str_val == "TRUE" || str_val == "1") {
    out_value = true;
    return true;
  } else if (str_val == "false" || str_val == "False" || str_val == "FALSE" ||
             str_val == "0") {
    out_value = false;
    return true;
  }
  last_error_ = "Failed to parse bool for key: " + key + ", value: " + str_val;
  return false;
}

std::vector<Waypoint> PerimeterLoader::parseWaypoints(
  const std::string& yaml, size_t start_idx)
{
  std::vector<Waypoint> waypoints;

  // Find the waypoints section
  size_t wp_start = findKey(yaml, "waypoints", start_idx);
  if (wp_start == std::string::npos) {
    last_error_ = "No waypoints found in YAML";
    return waypoints;
  }

  // Get the waypoints block (until next top-level key or end)
  size_t block_start = wp_start + 11;  // skip "waypoints:"

  // Find end of block (next key at column 0 that's not a list item)
  size_t block_end = yaml.size();
  size_t search_pos = block_start;
  while (search_pos < yaml.size()) {
    size_t line_start = yaml.find('\n', search_pos);
    if (line_start == std::string::npos) {
      break;
    }
    line_start++;

    // Check if this is end of file
    if (line_start >= yaml.size()) {
      break;
    }

    // Skip blank lines (they are valid within YAML lists)
    char c = yaml[line_start];
    if (c == '\r' || c == '\n') {
      search_pos = line_start;
      continue;
    }

    // Check if this line starts with a non-indented word (new key)
    // Blank lines and comment-only lines should NOT end the block
    if (yaml[line_start] != ' ' && yaml[line_start] != '\t' &&
        yaml[line_start] != '#') {
      // Make sure it's not a continuation of a list item
      block_end = line_start;
      break;
    }
    search_pos = line_start;
  }

  std::string wp_block = yaml.substr(block_start, block_end - block_start);
  std::istringstream stream(wp_block);
  std::string line;

  while (std::getline(stream, line)) {
    // Skip comments and empty lines
    if (isCommentOrEmpty(line)) {
      continue;
    }

    // Check if this is a list item (- {key: val, ...})
    std::string trimmed = trim(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    // Remove leading "- " or "  - "
    size_t content_start = 0;
    if (trimmed[0] == '-') {
      content_start = 1;
      while (content_start < trimmed.size() &&
             (trimmed[content_start] == ' ' || trimmed[content_start] == '\t')) {
        content_start++;
      }
    }

    if (content_start >= trimmed.size()) {
      continue;
    }

    // Parse inline map: {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
    std::string content = trimmed.substr(content_start);

    // Remove braces
    if (content[0] == '{') {
      content = content.substr(1);
    }
    if (!content.empty() && content.back() == '}') {
      content.pop_back();
    }

    Waypoint wp;
    wp.approach_radius = 1.0;  // default

    // Parse key-value pairs
    std::istringstream content_stream(content);
    std::string pair;
    while (std::getline(content_stream, pair, ',')) {
      std::string key_val = trim(pair);
      size_t colon = key_val.find(':');
      if (colon == std::string::npos) {
        continue;
      }

      std::string key = trim(key_val.substr(0, colon));
      std::string val = trim(key_val.substr(colon + 1));

      if (key == "x") {
        wp.x = std::stod(val);
      } else if (key == "y") {
        wp.y = std::stod(val);
      } else if (key == "heading") {
        wp.heading = std::stod(val);
      } else if (key == "radius") {
        wp.approach_radius = std::stod(val);
      }
    }

    waypoints.push_back(wp);
  }

  return waypoints;
}

PerimeterConfig PerimeterLoader::loadFromFile(const std::string& config_path)
{
  std::ifstream file(config_path);
  if (!file.is_open()) {
    last_error_ = "Failed to open file: " + config_path;
    std::cerr << "[PerimeterLoader] " << last_error_ << std::endl;
    return PerimeterConfig{};
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  return loadFromString(buffer.str());
}

PerimeterConfig PerimeterLoader::loadFromString(const std::string& yaml_content)
{
  PerimeterConfig config;
  last_error_ = "";

  // Check if this is a coverage config
  std::string type_val;
  if (extractValue(yaml_content, "type", type_val) && type_val == "coverage") {
    // Load as coverage configuration
    extractValue(yaml_content, "name", config.name);

    // Extract bounding box
    extractDouble(yaml_content, "min_x", config.bounding_box.min_x);
    extractDouble(yaml_content, "min_y", config.bounding_box.min_y);
    extractDouble(yaml_content, "max_x", config.bounding_box.max_x);
    extractDouble(yaml_content, "max_y", config.bounding_box.max_y);

    // Extract coverage parameters
    double pass_spacing = 2.0;
    if (extractDouble(yaml_content, "pass_spacing", pass_spacing)) {
      config.bounding_box.pass_spacing = pass_spacing;
    }

    double cov_speed = 1.0;
    if (extractDouble(yaml_content, "coverage_speed", cov_speed)) {
      config.bounding_box.coverage_speed = cov_speed;
    }

    std::string scan_str;
    if (extractValue(yaml_content, "scan_direction", scan_str) && !scan_str.empty()) {
      config.bounding_box.scan_direction = scan_str[0];
    }

    // Generate boustrophedon waypoints from coverage config
    CoverageConfig cov_cfg;
    cov_cfg.min_x = config.bounding_box.min_x;
    cov_cfg.min_y = config.bounding_box.min_y;
    cov_cfg.max_x = config.bounding_box.max_x;
    cov_cfg.max_y = config.bounding_box.max_y;
    cov_cfg.pass_spacing = config.bounding_box.pass_spacing;
    cov_cfg.coverage_speed = config.bounding_box.coverage_speed;
    cov_cfg.scan_direction = config.bounding_box.scan_direction;

    config.waypoints = generateBoustrophedonPattern(cov_cfg);
    config.closed_loop = false;  // Coverage is open path

    fprintf(stderr, "[LOADER] Loaded coverage config: %zu waypoints generated\n",
            config.waypoints.size());

    return config;
  }

  // Standard perimeter configuration
  extractValue(yaml_content, "name", config.name);

  // Extract closed_loop
  bool closed_loop = true;   // default
  extractBool(yaml_content, "closed_loop", closed_loop);
  config.closed_loop = closed_loop;

  // Extract tolerance
  double tolerance = 0.5;   // default
  extractDouble(yaml_content, "tolerance", tolerance);
  config.tolerance = tolerance;

  // Extract max_speed
  double max_speed = 2.0;   // default
  extractDouble(yaml_content, "max_speed", max_speed);
  config.max_speed = max_speed;

  // Extract min_turn_radius
  double min_turn_radius = 1.0;   // default
  extractDouble(yaml_content, "min_turn_radius", min_turn_radius);
  config.min_turn_radius = min_turn_radius;

  // Parse waypoints
  config.waypoints = parseWaypoints(yaml_content, 0);

  if (config.waypoints.empty()) {
    last_error_ = "No waypoints parsed from YAML";
    std::cerr << "[PerimeterLoader] " << last_error_ << std::endl;
  }

  return config;
}

std::vector<Waypoint> PerimeterLoader::generateBoustrophedonPattern(
  const CoverageConfig& config)
{
  std::vector<Waypoint> waypoints;

  const double epsilon = 1e-6;

  if (config.scan_direction == 'X') {
    // Horizontal passes (scan left-to-right, back-and-forth)
    const size_t num_passes = config.numPasses();

    for (size_t i = 0; i < num_passes; ++i) {
      double y = config.min_y + static_cast<double>(i) * config.pass_spacing;

      if (y > config.max_y + epsilon) {
        break;
      }

      if (i % 2 == 0) {
        // Even pass: left to right
        double x_start = config.min_x;
        double x_end = config.max_x;
        double heading = 0.0;  // pointing along +X

        waypoints.push_back({ x_start, y, heading, config.turn_arcs });
        waypoints.push_back({ x_end, y, heading, config.turn_arcs });
      } else {
        // Odd pass: right to left
        double x_start = config.max_x;
        double x_end = config.min_x;
        double heading = M_PI;  // pointing along -X

        waypoints.push_back({ x_start, y, heading, config.turn_arcs });
        waypoints.push_back({ x_end, y, heading, config.turn_arcs });
      }
    }

  } else {
    // Vertical passes (scan bottom-to-top, back-and-forth)
    const size_t num_passes = config.numPasses();

    for (size_t i = 0; i < num_passes; ++i) {
      double x = config.min_x + static_cast<double>(i) * config.pass_spacing;

      if (x > config.max_x + epsilon) {
        break;
      }

      if (i % 2 == 0) {
        // Even pass: bottom to top
        double y_start = config.min_y;
        double y_end = config.max_y;
        double heading = M_PI_2;  // pointing along +Y

        waypoints.push_back({ x, y_start, heading, config.turn_arcs });
        waypoints.push_back({ x, y_end, heading, config.turn_arcs });
      } else {
        // Odd pass: top to bottom
        double y_start = config.max_y;
        double y_end = config.min_y;
        double heading = -M_PI_2;  // pointing along -Y

        waypoints.push_back({ x, y_start, heading, config.turn_arcs });
        waypoints.push_back({ x, y_end, heading, config.turn_arcs });
      }
    }
  }

  return waypoints;
}

double CoverageConfig::computeCoveragePercent(
  const std::vector<Waypoint>& waypoints) const
{
  // Approximate coverage: total pass length / area
  double total_distance = 0.0;

  if (waypoints.size() < 2) {
    return 0.0;
  }

  for (size_t i = 1; i < waypoints.size(); ++i) {
    double dx = waypoints[i].x - waypoints[i - 1].x;
    double dy = waypoints[i].y - waypoints[i - 1].y;
    total_distance += std::hypot(dx, dy);
  }

  // Effective coverage width = pass_spacing * number_of_passes
  double effective_width = pass_spacing * static_cast<double>(waypoints.size() / 2);
  double area = width() * height();

  if (area <= 0.0) {
    return 0.0;
  }

  // Coverage ratio (pass width / pass_spacing, normalized to area)
  return std::min(100.0, (effective_width / pass_spacing) * 100.0);
}

}  // namespace perimeter_miner
