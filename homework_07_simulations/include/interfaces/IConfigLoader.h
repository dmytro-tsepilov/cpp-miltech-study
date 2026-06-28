#pragma once

#include <unordered_map>
#include <string>

struct AmmoType;
struct DroneConfig;

class IConfigLoader {
public:
    virtual ~IConfigLoader() = default;
    virtual bool load() = 0;

    virtual DroneConfig getConfig() = 0;
    virtual const std::unordered_map<std::string, AmmoType>& getAmmoParams() = 0;
};
