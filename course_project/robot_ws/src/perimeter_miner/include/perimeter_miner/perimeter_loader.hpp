// Copyright 2026 Open Source Robotics Foundation Inc
// SPDX-License-Identifier: MIT


#pragma once

#include <string>
#include <vector>

#include "perimeter_miner/perimeter_config.hpp"

namespace perimeter_miner {

/// YAML-based perimeter configuration loader
class PerimeterLoader {
public:
    /// Load perimeter config from YAML file
    /// @param config_path  Path to the YAML file (absolute or relative to install prefix)
    /// @return             Loaded PerimeterConfig, or empty config on failure
    static PerimeterConfig loadFromFile(const std::string& config_path);

    /// Load perimeter config from raw YAML string
    /// @param yaml_content  YAML string content
    /// @return              Loaded PerimeterConfig, or empty config on failure
    static PerimeterConfig loadFromString(const std::string& yaml_content);

    /// Get last error message (empty on success)
    static const std::string& getLastError() { return last_error_; }

private:
    /// Parse waypoints from YAML vector format
    static std::vector<Waypoint> parseWaypoints(const std::string& yaml, size_t start_idx);

    /// Simple YAML value extractor (handles simple key: value pairs)
    static bool extractValue(const std::string& yaml, const std::string& key,
                             std::string& out_value);

    /// Simple YAML double value extractor
    static bool extractDouble(const std::string& yaml, const std::string& key,
                              double& out_value);

    /// Simple YAML bool value extractor
    static bool extractBool(const std::string& yaml, const std::string& key, bool& out_value);

    /// Find a key in YAML string (handles indentation)
    static size_t findKey(const std::string& yaml, const std::string& key, size_t start = 0);

    static std::string last_error_;
};

}  // namespace perimeter_miner
