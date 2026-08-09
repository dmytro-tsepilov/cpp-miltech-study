#include "sensors/Mpu6050.h"

#include <cstdio>
#include <cstdint>
#include <iostream>
#include <unistd.h>

#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

const char* Mpu6050::name() const {
    return "MPU-6050";
}

bool Mpu6050::begin(int fd, uint8_t addr) {
    fd_ = fd;
    addr_ = addr;

    // Set device address on the bus
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
        std::cerr << "[MPU-6050] Failed to set I2C address 0x"
                  << std::hex << static_cast<int>(addr_) << std::dec
                  << ". Device may not be present on the bus.\n";
        return false;
    }

    // Verify WHO_AM_I register (0x75 should return 0x68)
    uint8_t reg = WHO_AM_I_REG;
    if (::write(fd_, &reg, 1) != 1) {
        std::cerr << "[MPU-6050] Failed to read WHO_AM_I register.\n";
        return false;
    }
    uint8_t whoAmI = 0;
    if (::read(fd_, &whoAmI, 1) != 1) {
        std::cerr << "[MPU-6050] Failed to read WHO_AM_I data.\n";
        return false;
    }

    if (whoAmI != WHO_AM_I_EXPECTED) {
        std::cerr << "[MPU-6050] WHO_AM_I mismatch: expected 0x"
                  << std::hex << static_cast<int>(WHO_AM_I_EXPECTED)
                  << ", got 0x" << static_cast<int>(whoAmI) << std::dec << "\n";
        return false;
    }
    std::printf("[MPU-6050] WHO_AM_I = 0x%02X — device verified.\n", whoAmI);

    // Configure: sleep=0, clock=auto gyro X, sample rate divider=0 (1kHz)
    uint8_t cfg[] = {REG_SMPLRT_DIV, 0x09};       // 1kHz / (9+1) = 100Hz
    if (::write(fd_, cfg, sizeof(cfg)) != static_cast<ssize_t>(sizeof(cfg))) return false;

    cfg[0] = REG_CONFIG;
    cfg[1] = 0x06; // DLPF config = 42Hz bandwidth
    if (::write(fd_, cfg, sizeof(cfg)) != static_cast<ssize_t>(sizeof(cfg))) return false;

    // Gyro config: FS_SEL=3 → ±2000 °/s
    cfg[0] = REG_GYRO_CONFIG;
    cfg[1] = 0x18; // bits [4:3] = 3
    if (::write(fd_, cfg, sizeof(cfg)) != static_cast<ssize_t>(sizeof(cfg))) return false;

    // Accel config: AFSEL=3 → ±16g
    cfg[0] = REG_ACCEL_CONFIG;
    cfg[1] = 0x18; // bits [4:3] = 3
    if (::write(fd_, cfg, sizeof(cfg)) != static_cast<ssize_t>(sizeof(cfg))) return false;

    // Clear sleep bit in PWR_MGMT_1
    cfg[0] = REG_PWR_MGMT_1;
    cfg[1] = 0x01; // CLKSEL = 0b001 (PLL w/ X gyro)
    if (::write(fd_, cfg, sizeof(cfg)) != static_cast<ssize_t>(sizeof(cfg))) return false;

    std::puts("[MPU-6050] Device configured successfully.");
    return true;
}

SensorReadings Mpu6050::read() {
    SensorReadings r{};
    r.valid = false;

    // Read 14 bytes: accel (6) + temp (2) + gyro (6) starting from 0x3B
    uint8_t reg = REG_ACCEL_XOUT_H;
    if (::write(fd_, &reg, 1) != 1) {
        std::cerr << "[MPU-6050] Failed to set data register pointer.\n";
        return r;
    }
    uint8_t data[14];
    if (::read(fd_, data, 14) != 14) {
        std::cerr << "[MPU-6050] Failed to read sensor data.\n";
        return r;
    }

    // Assemble 16-bit big-endian values
    int16_t accel_x = (data[0] << 8) | data[1];
    int16_t accel_y = (data[2] << 8) | data[3];
    int16_t accel_z = (data[4] << 8) | data[5];
    int16_t temp_raw = (data[6] << 8) | data[7];
    int16_t gyro_x = (data[8] << 8) | data[9];
    int16_t gyro_y = (data[10] << 8) | data[11];
    int16_t gyro_z = (data[12] << 8) | data[13];

    // Convert to physical units
    double accel_x_g = static_cast<double>(accel_x) / ACCEL_SENS[3]; // ±16g
    double accel_y_g = static_cast<double>(accel_y) / ACCEL_SENS[3];
    double accel_z_g = static_cast<double>(accel_z) / ACCEL_SENS[3];

    double gyro_x_dps = static_cast<double>(gyro_x) / (static_cast<double>(GYRO_SENS[3]) / 1.0);
    double gyro_y_dps = static_cast<double>(gyro_y) / (static_cast<double>(GYRO_SENS[3]) / 1.0);
    double gyro_z_dps = static_cast<double>(gyro_z) / (static_cast<double>(GYRO_SENS[3]) / 1.0);

    // Temperature: T(°C) = raw/340.0 + 36.53
    double temp_c = static_cast<double>(temp_raw) / 340.0 + 36.53;

    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "Accel: X=%+6.3fg Y=%+6.3fg Z=%+6.3fg  |  "
        "Gyro:  X=%+7.1f°/s Y=%+7.1f°/s Z=%+7.1f°/s  |  "
        "Temp: %+.1f°C",
        accel_x_g, accel_y_g, accel_z_g,
        gyro_x_dps, gyro_y_dps, gyro_z_dps,
        temp_c);
    r.output = buf;
    r.label = "Accel/Gyro/Temp";
    r.valid = true;
    return r;
}
