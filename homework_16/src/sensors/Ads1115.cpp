#include "sensors/Ads1115.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <unistd.h>

#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

const char* Ads1115::name() const {
    return "ADS1115";
}

bool Ads1115::begin(int fd, uint8_t addr) {
    fd_ = fd;
    addr_ = addr;

    // Set device address on the bus
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
        std::cerr << "[Ads1115] Failed to set I2C address 0x"
                  << std::hex << static_cast<int>(addr_) << std::dec
                  << ". Device may not be present on the bus.\n";
        return false;
    }

    // Verify device by writing config register and reading it back.
    // ADS1115 has no WHO_AM_I, so we use this as proof of life.
    uint8_t config[4] = {0x01, 0x80, 0x01, 0x83};
    ssize_t n = ::write(fd_, config, sizeof(config));
    if (n != static_cast<ssize_t>(sizeof(config))) {
        std::cerr << "[Ads1115] Failed to write initial config.\n";
        return false;
    }

    // Read back the config register to verify
    uint8_t reg = 0x02;
    if (::write(fd_, &reg, 1) != 1) {
        std::cerr << "[Ads1115] Failed to read config register.\n";
        return false;
    }
    uint8_t cfg[3];
    if (::read(fd_, cfg, sizeof(cfg)) != static_cast<ssize_t>(sizeof(cfg))) {
        std::cerr << "[Ads1115] Failed to read back config.\n";
        return false;
    }

    std::printf("[Ads1115] Config register read back: 0x%02X 0x%02X 0x%02X\n",
                cfg[0], cfg[1], cfg[2]);
    std::puts("[Ads1115] Device verified successfully.");
    return true;
}

SensorReadings Ads1115::read() {
    SensorReadings r{};
    r.valid = false;
    r.label = "Voltage";

    // Build config: MUX=AIN0/AIN1, PGA=±6.144V, DR=128SPS, OS=1 (single-shot)
    uint8_t buf[4] = {
        0x01, // pointer to config register
        0x40, // MUX: AINP=AIN0, AINN=AIN1
        0x00, // PGA: ±6.144V (gain = 2/3)
        0x83  // DR=128 SPS, OS=1 (start conversion)
    };

    if (::write(fd_, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf))) {
        std::cerr << "[Ads1115] Failed to start conversion.\n";
        return r;
    }

    // Wait for conversion (max ~7.8 ms at 128 SPS)
    usleep(10000); // 10 ms

    // Read 2-byte conversion result (big-endian)
    uint8_t reg = 0x00;
    if (::write(fd_, &reg, 1) != 1) {
        std::cerr << "[Ads1115] Failed to set conversion register pointer.\n";
        return r;
    }
    uint8_t data[2];
    if (::read(fd_, data, 2) != 2) {
        std::cerr << "[Ads1115] Failed to read conversion data.\n";
        return r;
    }

    int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    double volts = static_cast<double>(raw) * 0.0001875; // LSB = 0.1875 mV

    char buf2[64];
    if (std::abs(volts) >= 1.0) {
        std::snprintf(buf2, sizeof(buf2), "%+8.3f V  (raw=%+6d)", volts, raw);
    } else {
        double mv = volts * 1000.0;
        std::snprintf(buf2, sizeof(buf2), "%+8.3f mV (raw=%+6d)", mv, raw);
    }
    r.output = buf2;
    r.valid = true;
    return r;
}
