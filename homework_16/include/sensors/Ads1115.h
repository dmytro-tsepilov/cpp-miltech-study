#pragma once

#include <cstdint>
#include "../interfaces/ISensor.h"

// ============================================================
// ADS1115 — 16-bit ADC driver (address 0x48)
// ============================================================
class Ads1115 : public ISensor {
public:
    static constexpr uint8_t ADDR = 0x48;

    const char* name() const override;
    bool begin(int fd, uint8_t addr) override;
    SensorReadings read() override;

private:
    int fd_ = -1;
    uint8_t addr_ = 0;
};
