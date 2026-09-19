// Copyright 2026 Perimeter Miner Project
// SPDX-License-Identifier: MIT

#include "perimeter_miner/perimeter_loader.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

namespace perimeter_miner {

// Helper: trim whitespace
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

// Helper: check if line is a comment or empty
static bool isCommentOrEmpty(const std::string& line) {
    std::string trimmed = trim(line);
    return trimmed.empty() || trimmed[0] == '#';
}

size_t PerimeterLoader::findKey(const std::string& yaml, const std::string& key, size_t start) {
    // Look for "key:" pattern in yaml starting from 'start'
    std::string pattern = key + ":";
    size_t pos = yaml.find(pattern, start);

    while (pos != std::string::npos) {
        // Verify it's a proper key (preceded by whitespace or start of line)
        if (pos == 0 || yaml[pos - 1] == ' ' || yaml[pos - 1] == '\t') {
            return pos;
        }
        pos = yaml.find(pattern, pos + 1);
    }
    return std::string::npos;
}

bool PerimeterLoader::extractValue(const std::string& yaml, const std::string& key, std::string& out_value) {
    size_t pos = findKey(yaml, key);
    if (pos == std::string::npos) return false;

    // Get the value after "key:"
    size_t val_start = pos + key.size() + 1; // skip "key:"
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

bool PerimeterLoader::extractDouble(const std::string& yaml, const std::string& key, double& out_value) {
    std::string str_val;
    if (!extractValue(yaml, key, str_val)) return false;

    try {
        out_value = std::stod(str_val);
        return true;
    } catch (...) {
        last_error_ = "Failed to parse double for key: " + key;
        return false;
    }
}

bool PerimeterLoader::extractBool(const std::string& yaml, const std::string& key, bool& out_value) {
    std::string str_val;
    if (!extractValue(yaml, key, str_val)) return false;

    if (str_val == "true" || str_val == "True" || str_val == "TRUE" || str_val == "1") {
        out_value = true;
        return true;
    } else if (str_val == "false" || str_val == "False" || str_val == "FALSE" || str_val == "0") {
        out_value = false;
        return true;
    }
    last_error_ = "Failed to parse bool for key: " + key + ", value: " + str_val;
    return false;
}

std::vector<Waypoint> PerimeterLoader::parseWaypoints(const std::string& yaml, size_t start_idx) {
    std::vector<Waypoint> waypoints;

    // Find the waypoints section
    size_t wp_start = findKey(yaml, "waypoints", start_idx);
    if (wp_start == std::string::npos) {
        last_error_ = "No waypoints found in YAML";
        return waypoints;
    }

    // Get the waypoints block (until next top-level key or end)
    size_t block_start = wp_start + 11; // skip "waypoints:"

    // Find end of block (next key at column 0 that's not a list item)
    size_t block_end = yaml.size();
    size_t search_pos = block_start;
    while (search_pos < yaml.size()) {
        size_t line_start = yaml.find('\n', search_pos);
        if (line_start == std::string::npos) break;
        line_start++;

        // Check if this line starts with a non-indented word (new key)
        if (line_start < yaml.size() && yaml[line_start] != ' ' && yaml[line_start] != '\t' &&
            yaml[line_start] != '#' && yaml[line_start] != '-') {
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
        if (isCommentOrEmpty(line)) continue;

        // Check if this is a list item (- {key: val, ...})
        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') continue;

        // Remove leading "- " or "  - "
        size_t content_start = 0;
        if (trimmed[0] == '-') {
            content_start = 1;
            while (content_start < trimmed.size() && (trimmed[content_start] == ' ' || trimmed[content_start] == '\t')) {
                content_start++;
            }
        }

        if (content_start >= trimmed.size()) continue;

        // Parse inline map: {x: 0.0, y: 0.0, heading: 0.0, radius: 1.0}
        std::string content = trimmed.substr(content_start);

        // Remove braces
        if (content[0] == '{') content = content.substr(1);
        if (!content.empty() && content.back() == '}') content.pop_back();

        Waypoint wp;
        wp.approach_radius = 1.0; // default

        // Parse key-value pairs
        std::istringstream content_stream(content);
        std::string pair;
        while (std::getline(content_stream, pair, ',')) {
            std::string key_val = trim(pair);
            size_t colon = key_val.find(':');
            if (colon == std::string::npos) continue;

            std::string key = trim(key_val.substr(0, colon));
            std::string val = trim(key_val.substr(colon + 1));

            if (key == "x") wp.x = std::stod(val);
            else if (key == "y") wp.y = std::stod(val);
            else if (key == "heading") wp.heading = std::stod(val);
            else if (key == "radius") wp.approach_radius = std::stod(val);
        }

        waypoints.push_back(wp);
    }

    return waypoints;
}

PerimeterConfig PerimeterLoader::loadFromFile(const std::string& config_path) {
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

PerimeterConfig PerimeterLoader::loadFromString(const std::string& yaml_content) {
    PerimeterConfig config;
    last_error_ = "";

    // Extract name
    std::string name_val;
    if (extractValue(yaml_content, "name", name_val)) {
        config.name = name_val;
    } else {
        config.name = "unnamed";
    }

    // Extract closed_loop
    bool closed_loop = true; // default
    extractBool(yaml_content, "closed_loop", closed_loop);
    config.closed_loop = closed_loop;

    // Extract tolerance
    double tolerance = 0.5; // default
    extractDouble(yaml_content, "tolerance", tolerance);
    config.tolerance = tolerance;

    // Extract max_speed
    double max_speed = 2.0; // default
    extractDouble(yaml_content, "max_speed", max_speed);
    config.max_speed = max_speed;

    // Extract min_turn_radius
    double min_turn_radius = 1.0; // default
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

} // namespace perimeter_miner
