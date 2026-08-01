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

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!tracks_.empty()) {
                break;
            }
        }
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > timeout) {
            LOG("UartTargetProvider::load() — timeout waiting for targets");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    threadReady_ = true;
    LOG("UartTargetProvider::load() — targets received");
    return true;
}

int UartTargetProvider::getTargetCount()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(tracks_.size());
}

int UartTargetProvider::getTimeSteps()
{
    return timeSteps_;
}

Target UartTargetProvider::getTarget(int index)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < 0 || index >= static_cast<int>(tracks_.size())) {
        LOG("UartTargetProvider::getTarget() — index " << index << " out of range");
        return Target{};
    }

    const auto& tr = tracks_[index];
    // Return current position with the causally estimated velocity/acceleration
    // so MissionProcessor::leadTarget can predict the target's future position.
    return Target{ tr.pos, tr.velocity, tr.acceleration };
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

    int idx = static_cast<int>(pos.id);
    if (idx < 0) {
        return;
    }
    if (idx + 1 > static_cast<int>(tracks_.size())) {
        tracks_.resize(idx + 1);
    }

    TargetTrack& tr = tracks_[idx];
    const Coord newPos{ pos.x, pos.y };

    // Stamp the sample with the checker's SIMULATION time (TELEMETRY t_ms), so the
    // finite-difference velocity is in sim m/s — exact and independent of wall-clock
    // jitter and timeScale. TARGET packets are sent together with TELEMETRY each tick,
    // so the latest telemetry timestamp is the correct time for this target sample.
    //
    // NOTE: this runs inside the UART RX thread's TARGET callback, which is invoked
    // while the telemetry provider holds its mutex. Use the LOCK-FREE timestamp
    // accessor here — calling getTelemetry() would re-lock that same mutex and
    // self-deadlock the RX thread.
    auto* provider = uartTel_ ? uartTel_ : g_uartTel;
    const double nowSec = provider ? (provider->getTelemetryTimeMs() / 1000.0) : tr.lastSimSec;

    if (!tr.hasPos) {
        // First sample for this target: position only, velocity stays zero.
        tr.pos = newPos;
        tr.lastSimSec = nowSec;
        tr.hasPos = true;
    } else {
        const double simDt = nowSec - tr.lastSimSec;

        if (simDt > 1e-3) {
            const Coord vel = (newPos - tr.pos) / simDt;
            const float alpha = 0.5f;  // EMA smoothing to tame finite-difference noise
            const Coord smoothVel = tr.hasVel ? (vel * alpha + tr.velocity * (1.0f - alpha)) : vel;

            if (tr.hasVel) {
                const Coord acc = (smoothVel - tr.velocity) / simDt;
                tr.acceleration = acc * alpha + tr.acceleration * (1.0f - alpha);
            }

            tr.velocity = smoothVel;
            tr.hasVel = true;
            tr.pos = newPos;
            tr.lastSimSec = nowSec;
        }
        // else: interval too small to difference reliably — ignore this sample.
    }

    targetCount_ = std::max(targetCount_, idx + 1);

    LOG("UartTargetProvider: TARGET[" << static_cast<int>(pos.id) << "] = ("
            << pos.x << ", " << pos.y << ") vel=("
            << tr.velocity.x << ", " << tr.velocity.y << ")");
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
