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

#pragma once

#include <string>
#include <vector>

#include "perimeter_miner/perimeter_config.hpp"

namespace perimeter_miner
{

/// YAML-based perimeter configuration loader
class PerimeterLoader
{
public:
  /// Load perimeter config from YAML file
  static PerimeterConfig loadFromFile(const std::string &config_path);

  /// Load perimeter config from YAML string
  static PerimeterConfig loadFromString(const std::string &yaml_content);

  /// Get last error message
  static const std::string &getLastError() { return last_error_; }

private:
  /// Find a key in YAML content
  static size_t findKey(const std::string &yaml, const std::string &key, size_t start = 0);

  /// Extract string value for a key
  static bool extractValue(const std::string &yaml, const std::string &key, std::string &out_value);

  /// Extract double value for a key
  static bool extractDouble(const std::string &yaml, const std::string &key, double &out_value);

  /// Extract bool value for a key
  static bool extractBool(const std::string &yaml, const std::string &key, bool &out_value);

  /// Parse waypoints from YAML content
  static std::vector<Waypoint> parseWaypoints(const std::string &yaml, size_t start_idx = 0);

  /// Last error message
  static std::string last_error_;
};

} // namespace perimeter_miner
