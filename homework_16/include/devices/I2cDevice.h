#pragma once

#include <cstdint>
#include <string>
#include <string_view>

class I2cDevice {
public:
    explicit I2cDevice(std::string_view path);

    ~I2cDevice();

    // Non-copyable
    I2cDevice(const I2cDevice&) = delete;
    I2cDevice& operator=(const I2cDevice&) = delete;

    bool open();
    void close();
    bool setAddress(uint8_t addr);
    bool writeReg(uint8_t reg, uint8_t value);
    bool readReg(uint8_t reg, uint8_t* buf, size_t len);
    int getFd() const;

private:
    int fd_;
    std::string path_;
};
