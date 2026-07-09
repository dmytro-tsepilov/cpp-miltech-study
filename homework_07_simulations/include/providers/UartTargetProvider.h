#pragma once

#include <vector>
#include <mutex>
#include <atomic>
#include <string>
#include <chrono>

#include "interfaces/ITargetProvider.h"
#include "config/DroneConfig.h"
#include "protocol/IUartTelemetryProvider.h"
#include "protocol/drone_link.h"
#include "common/macros.h"

// UartTargetProvider — implements ITargetProvider by reading TARGET packets
// from IUartTelemetryProvider. UART sends individual target positions (not full
// trajectories), so this provider CAUSALLY estimates each target's velocity and
// acceleration from consecutive TARGET packets (backward finite differences),
// restoring the lead-targeting that the file-based provider gets from trajectory
// look-ahead — without peeking into the future.
class UartTargetProvider : public ITargetProvider {
private:
    // Per-target causal state: last position/velocity + timestamp for differencing.
    // The timestamp is the checker's SIMULATION time (from TELEMETRY t_ms), not
    // wall-clock — this makes the velocity/acceleration estimate exact and free of
    // scheduler jitter and timeScale scaling.
    struct TargetTrack {
        Coord  pos{0, 0};
        Coord  velocity{0, 0};
        Coord  acceleration{0, 0};
        bool   hasPos = false;
        bool   hasVel = false;
        double lastSimSec = 0.0;   // sim time of the last sample, seconds
    };

    // Global pointer set once by main.cpp after creating the telemetry provider
    static IUartTelemetryProvider* g_uartTel;

    // Active instance pointer for static packet handlers
    static UartTargetProvider* s_activeInstance;

    IUartTelemetryProvider* uartTel_{nullptr};

    std::atomic<bool> threadReady_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_{false};

    int targetCount_{0};
    int timeSteps_{1};  // UART provides real-time positions, not pre-recorded trajectories

    // Per-target tracks (updated from PKT_TARGET packets)
    mutable std::mutex mutex_;
    std::vector<TargetTrack> tracks_;

    float arrayTimeStep_{1.0f};
    float timeScale_{1.0f};

public:
    UartTargetProvider() = default;
    ~UartTargetProvider() override = default;

    // Set the global UART telemetry provider (called once by main.cpp)
    static void setGlobalUartTelemetryProvider(IUartTelemetryProvider* provider) {
        g_uartTel = provider;
    }

    // Set the active instance pointer for static packet handlers
    static void setInstance(UartTargetProvider* inst) {
        s_activeInstance = inst;
    }

    // ITargetProvider interface
    bool load() override;
    int getTargetCount() override;
    int getTimeSteps() override;
    Target getTarget(int index) override;

    void setTimings(float arrayTimeStep, float timeScale) override;
    void run() override;
    void start() override;
    void stop() override;
    bool isThreadReady() override;

    // Called internally when TARGET packet arrives
    void onTargetReceived(const dlink::TargetPos& pos);

    // Static callback function for packet dispatch
    static void onTargetPacket(const uint8_t* payload, uint8_t len);

    // Register this provider's callbacks with the UART telemetry provider
    static void registerCallbacks();
};
