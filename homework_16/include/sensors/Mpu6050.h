#pragma once

#include <cstdint>
#include "../interfaces/ISensor.h"

// ============================================================
// MPU-6050 — accelerometer + gyroscope driver (address 0x68)
// ============================================================
class Mpu6050 : public ISensor {
public:
    static constexpr uint8_t ADDR = 0x68;
    static constexpr uint8_t WHO_AM_I_REG = 0x75;
    static constexpr uint8_t WHO_AM_I_EXPECTED = 0x68;

    // Register addresses
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_TEMP_OUT_H     = 0x41;
    static constexpr uint8_t REG_GYRO_XOUT_H    = 0x43;
    static constexpr uint8_t REG_PWR_MGMT_1     = 0x6B;
    static constexpr uint8_t REG_SMPLRT_DIV     = 0x19;
    static constexpr uint8_t REG_CONFIG         = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG    = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG   = 0x1C;

    // Gyro full-scale: 0 = ±250, 1 = ±500, 2 = ±1000, 3 = ±2000 °/s
    static constexpr int16_t GYRO_SENS[] = {65536/250, 65536/500, 65536/1000, 65536/2000};
    // Accel full-scale: 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g
    static constexpr int16_t ACCEL_SENS[]  = {16384, 8192, 4096, 2048};

    const char* name() const override;
    bool begin(int fd, uint8_t addr) override;
    SensorReadings read() override;

private:
    int fd_ = -1;
    uint8_t addr_ = 0;
};
