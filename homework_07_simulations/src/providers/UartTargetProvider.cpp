#include "providers/UartTargetProvider.h"
#include "common/macros.h"
#include "protocol/IUartTelemetryProvider.h"

#include <algorithm>
#include <cstring>
#include <thread>
#include <chrono>

// Static member definitions
IUartTelemetryProvider* UartTargetProvider::g_uartTel = nullptr;
UartTargetProvider* UartTargetProvider::s_activeInstance = nullptr;

// Forward declaration of global callback registration (defined in UartTelemetryProvider.cpp)
using GlobalPacketCallback = void(*)(const uint8_t*, uint8_t);
extern void uart_telemetry_register_global_callback(dlink::PacketType type, GlobalPacketCallback cb);

// ---- ITargetProvider implementation ----

bool UartTargetProvider::load()
{
    // Use global pointer if instance pointer not set
    auto* provider = uartTel_ ? uartTel_ : g_uartTel;
    if (!provider) {
        LOG("UartTargetProvider::load() — no UART telemetry provider set");
        return false;
    }

    // Wait for at least one TARGET packet to arrive (or use nTargets from AMMO)
    auto timeout = std::chrono::seconds(30);
    auto start = std::chrono::steady_clock::now();

    while (currentTargets_.empty()) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > timeout) {
            LOG("UartTargetProvider::load() — timeout waiting for targets");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    threadReady_ = true;
    LOG("UartTargetProvider::load() — " << currentTargets_.size() << " targets received");
    return true;
}

int UartTargetProvider::getTargetCount()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(currentTargets_.size());
}

int UartTargetProvider::getTimeSteps()
{
    return timeSteps_;
}

Target UartTargetProvider::getTarget(int index)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < 0 || index >= static_cast<int>(currentTargets_.size())) {
        LOG("UartTargetProvider::getTarget() — index " << index << " out of range");
        return Target{};
    }

    const auto& pos = currentTargets_[index];
    // Return current position with zero velocity (UART doesn't provide velocity)
    return Target{
        Coord{pos.x, pos.y},
        Coord{0, 0}
    };
}

void UartTargetProvider::setTimings(float arrayTimeStep, float timeScale)
{
    arrayTimeStep_ = arrayTimeStep;
    timeScale_ = timeScale;
}

void UartTargetProvider::run()
{
    // UART targets arrive asynchronously via onTargetReceived — no loop needed
    threadReady_ = true;

    while (!stop_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void UartTargetProvider::start()
{
    started_ = true;
}

void UartTargetProvider::stop()
{
    stop_ = true;
}

bool UartTargetProvider::isThreadReady()
{
    return threadReady_;
}

void UartTargetProvider::onTargetReceived(const dlink::TargetPos& pos)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Resize vector if needed (targets may arrive incrementally)
    int idx = static_cast<int>(pos.id);
    if (idx >= 0) {
        if (idx + 1 >= static_cast<int>(currentTargets_.size())) {
            currentTargets_.resize(idx + 1);
        }
        currentTargets_[idx] = pos;
        targetCount_ = std::max(targetCount_, idx + 1);

        LOG("UartTargetProvider: TARGET[" << pos.id << "] = (" << pos.x << ", " << pos.y << ")");
    }
}

// ---- Static packet handlers ----

void UartTargetProvider::onTargetPacket(const uint8_t* payload, uint8_t len)
{
    if (len != sizeof(dlink::TargetPos)) return;
    
    if (s_activeInstance) {
        dlink::TargetPos pos;
        std::memcpy(&pos, payload, len);
        s_activeInstance->onTargetReceived(pos);
    }
}

void UartTargetProvider::registerCallbacks()
{
    // Register global callback with the UART telemetry provider
    uart_telemetry_register_global_callback(dlink::PKT_TARGET, &UartTargetProvider::onTargetPacket);
}
