#pragma once

#include <string>
#include <vector>
#include <random>

namespace mine_simulator {

/// Mine definition for simulation
struct MineDefinition {
    int id = 0;
    double x = 0.0;
    double y = 0.0;
    std::string type;  // "anti-tank", "anti-personnel", "unknown"
    bool detected = false;
    bool cleared = false;
};

/// Mine simulation configuration
struct MineSimConfig {
    std::string name;
    std::vector<MineDefinition> mines;
    
    // Detection parameters
    double detection_range = 3.0;       // meters
    double detection_probability = 0.95; // base probability
    double detection_decay = 0.5;        // probability decreases with distance
    
    // Simulation timing
    double update_period_ms = 100.0;    // ms between updates
};

/// Load mine configuration from YAML (simplified parser)
MineSimConfig loadMineConfig(const std::string& path);

/// Save mine configuration to YAML (simplified writer)
bool saveMineConfig(const MineSimConfig& config, const std::string& path);

} // namespace mine_simulator
