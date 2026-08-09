#include <cstdint>
#include <cstdio>
#include <iostream>
#include <signal.h>
#include <memory>

#include "devices/I2cDevice.h"
#include "interfaces/ISensor.h"
#include "sensors/Ads1115.h"
#include "sensors/Mpu6050.h"

// ============================================================
// Global flag for graceful shutdown
// ============================================================
static volatile sig_atomic_t g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

// ============================================================
// Sensor factory — auto-detect by address
// ============================================================
static std::unique_ptr<ISensor> createSensor(uint8_t addr) {
    if (addr == Ads1115::ADDR) {
        return std::make_unique<Ads1115>();
    }
    if (addr == Mpu6050::ADDR) {
        return std::make_unique<Mpu6050>();
    }
    return nullptr;
}

// ============================================================
// Helpers
// ============================================================
static void printHelp(std::string_view prog) {
    std::printf("Usage: %s <i2c_device> <address>\n", prog.data());
    std::printf("  i2c_device  e.g. /dev/i2c-1\n");
    std::printf("  address     I2C device address in hex:\n");
    std::printf("              0x48 → ADS1115 (16-bit ADC, voltage)\n");
    std::printf("              0x68 → MPU-6050 (accelerometer + gyroscope)\n");
    std::printf("\nExamples:\n");
    std::printf("  LD_PRELOAD=./libi2csim.so %s /dev/i2c-1 0x48\n", prog.data());
    std::printf("  LD_PRELOAD=./libi2csim.so %s /dev/i2c-1 0x68\n", prog.data());
}

// ============================================================
// main
// ============================================================
int main(int argc, char* argv[]) {
    // Install signal handler for graceful shutdown
    struct sigaction sa{};
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    if (argc < 3) {
        printHelp(argv[0]);
        return EXIT_FAILURE;
    }

    std::string devPath = argv[1];
    uint8_t deviceAddr;

    // Parse hex address
    char* endptr = nullptr;
    long addrVal = std::strtol(argv[2], &endptr, 16);
    if (endptr == argv[2] || addrVal < 0 || addrVal > 0x7F) {
        std::cerr << "Error: invalid I2C address '" << argv[2] << "'\n";
        return EXIT_FAILURE;
    }
    deviceAddr = static_cast<uint8_t>(addrVal);

    // Detect sensor type
    auto sensor = createSensor(deviceAddr);
    if (!sensor) {
        std::cerr << "Error: unsupported device address 0x"
                  << std::hex << static_cast<int>(deviceAddr) << std::dec
                  << ". Supported: 0x48 (ADS1115), 0x68 (MPU-6050)\n";
        return EXIT_FAILURE;
    }

    std::printf("I2C %s Reader\n", sensor->name());
    std::printf("  Device : %s\n", devPath.c_str());
    std::printf("  Address: 0x%02X\n", deviceAddr);
    std::puts("");

    // Open I2C bus
    I2cDevice i2c(devPath);
    if (!i2c.open()) {
        std::cerr << "Error: cannot open " << devPath << "\n";
        return EXIT_FAILURE;
    }

    // Initialize sensor
    if (!sensor->begin(i2c.getFd(), deviceAddr)) {
        std::cerr << "Error: " << sensor->name() << " not found at 0x"
                  << std::hex << static_cast<int>(deviceAddr) << std::dec << "\n";
        return EXIT_FAILURE;
    }

    std::printf("Starting %s readings (Ctrl+C to stop)...\n", sensor->name());
    std::puts("-----------------------------------------");

    int count = 0;
    while (g_running) {
        SensorReadings r = sensor->read();
        if (!r.valid) {
            std::cerr << "\nError: failed to read " << sensor->name() << ".\n";
            break;
        }

        count++;
        std::printf("[%d] %s\n", count, r.output.c_str());

        // Sleep ~250ms for ~4 Hz update rate
        usleep(250000);
    }

    std::puts("\n-----------------------------------------");
    std::printf("Stopped after %d readings.\n", count);

    return EXIT_SUCCESS;
}
