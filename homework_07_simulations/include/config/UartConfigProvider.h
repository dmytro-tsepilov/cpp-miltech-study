#pragma once

#include <unordered_map>
#include <string>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include "interfaces/IConfigLoader.h"
#include "config/DroneConfig.h"
#include "protocol/IUartTelemetryProvider.h"
#include "protocol/drone_link.h"

// UartConfigProvider — implements IConfigLoader by reading CONFIG and AMMO
// packets from IUartTelemetryProvider instead of JSON files.
class UartConfigProvider : public IConfigLoader {
private:
    // Global pointer set once by main.cpp after creating the telemetry provider
    static IUartTelemetryProvider* g_uartTel;

    // Active instance pointer for static packet handlers
    static UartConfigProvider* s_activeInstance;

    IUartTelemetryProvider* uartTel_{nullptr};

    // Cached data from UART packets
    std::atomic<bool> configReceived_{false};
    std::atomic<bool> ammoReceived_{false};
    std::atomic<bool> loadComplete_{false};

    DroneConfig droneConfig_;
    std::unordered_map<std::string, AmmoType> ammoTypes_;

    // Synchronization for load() blocking
    std::mutex cvMutex_;
    std::condition_variable cv_;

    void convertDroneCfg(const dlink::DroneCfg& cfg);
    void convertAmmoCfg(const dlink::AmmoCfg& ammo);

public:
    UartConfigProvider() = default;
    ~UartConfigProvider() override = default;

    // Set the global UART telemetry provider (called once by main.cpp)
    static void setGlobalUartTelemetryProvider(IUartTelemetryProvider* provider) {
        g_uartTel = provider;
    }

    // Set the active instance pointer for static packet handlers
    static void setInstance(UartConfigProvider* inst) {
        s_activeInstance = inst;
    }

    // IConfigLoader interface
    bool load() override;
    DroneConfig getConfig() override { return droneConfig_; }
    const std::unordered_map<std::string, AmmoType>& getAmmoParams() override {
        return ammoTypes_;
    }

    // Called internally when UART packets arrive
    void onConfigReceived(const dlink::DroneCfg& cfg);
    void onAmmoReceived(const dlink::AmmoCfg& ammo);

    // Static callback functions for packet dispatch
    static void onConfigPacket(const uint8_t* payload, uint8_t len);
    static void onAmmoPacket(const uint8_t* payload, uint8_t len);

    // Register this provider's callbacks with the UART telemetry provider
    static void registerCallbacks();
};
