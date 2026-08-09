#pragma once

#include <cstdint>
#include <string>

struct SensorReadings {
    bool valid = false;
    std::string label;
    std::string output;
};

class ISensor {
public:
    virtual ~ISensor() = default;
    virtual bool begin(int fd, uint8_t addr) = 0;
    virtual SensorReadings read() = 0;
    virtual const char* name() const = 0;
};
