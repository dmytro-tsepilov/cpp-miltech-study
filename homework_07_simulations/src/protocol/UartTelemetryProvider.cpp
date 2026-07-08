#include "protocol/IUartTelemetryProvider.h"
#include "protocol/IUartLink.h"
#include "protocol/IDroneGpioController.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cstring>
#include <vector>

class UartTelemetryProvider : public IUartTelemetryProvider {
private:
    IUartLink* uart_ = nullptr;
    IDroneGpioController* gpio_ = nullptr;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> ready_{false};
    std::atomic<bool> simRunning_{false};

    mutable std::mutex mutex_;
    dlink::Telemetry lastTel_{};
    dlink::TargetPos lastTarget_{};
    dlink::AmmoCfg lastAmmo_{};

    dlink::Parser parser_;

    // Subscriber callbacks for specific packet types
    using PacketCallback = void(*)(const uint8_t*, uint8_t);
    struct CallbackEntry {
        dlink::PacketType type;
        PacketCallback callback;
    };
    std::vector<CallbackEntry> callbacks_;

    // Global callbacks registered by UART-based providers
    static std::vector<CallbackEntry> g_globalCallbacks;

    void dispatchCallbacks(dlink::PacketType type, const uint8_t* payload, uint8_t len) {
        // First dispatch global callbacks (from UART-based providers)
        for (auto& cb : g_globalCallbacks) {
            if (cb.type == type && cb.callback) {
                cb.callback(payload, len);
            }
        }
        // Then dispatch instance-specific callbacks
        for (auto& cb : callbacks_) {
            if (cb.type == type && cb.callback) {
                cb.callback(payload, len);
            }
        }
    }

    // Use same type as interface for override compatibility
    using GlobalPacketCallback = PacketCallback;

    void runLoop() {
        uint8_t payloadBuf[260];
        uint8_t typeByte = 0;
        uint8_t lenByte = 0;

        // Wait for START line to be HIGH (checker sees our START signal)
        std::cout << "[TelProv] Waiting for checker START signal..." << std::endl;
        while (running_) {
            if (gpio_ && gpio_->isReady()) {
                // In sim mode, we need to check the GPIO state from checker's perspective.
                // Since we control START, it should already be HIGH.
                // The checker will start sending telemetry once it sees START=1.
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        simRunning_ = true;
        std::cout << "[TelProv] START seen, waiting for first telemetry..." << std::endl;

        while (running_) {
            // Read available bytes from UART
            uint8_t buf[256];
            int n = uart_->readBytes(buf, static_cast<int>(sizeof(buf)));
            //std::cout << "[TelProv] bytes read: " << n << std::endl;

            if (n > 0) {
                for (int i = 0; i < n; ++i) {
                    if (parser_.feed(buf[i], typeByte, payloadBuf, lenByte)) {
                        dlink::PacketType pktType = static_cast<dlink::PacketType>(typeByte);

                        switch (pktType) {
                            case dlink::PKT_TELEMETRY: {
                                if (lenByte == sizeof(dlink::Telemetry)) {
                                    std::lock_guard<std::mutex> lock(mutex_);
                                    std::memcpy(&lastTel_, payloadBuf, lenByte);
                                    ready_ = true;
                                    std::cout << "[TelProv] TELEMETRY: t=" << lastTel_.t_ms
                                              << " pos=(" << lastTel_.x << "," << lastTel_.y
                                              << ") z=" << lastTel_.z
                                              << " speed=" << lastTel_.speed
                                              << " dir=" << lastTel_.dir
                                              << " state=" << static_cast<int>(lastTel_.state)
                                              << std::endl;
                                }
                                break;
                            }
                            case dlink::PKT_TARGET: {
                                if (lenByte == sizeof(dlink::TargetPos)) {
                                    std::lock_guard<std::mutex> lock(mutex_);
                                    std::memcpy(&lastTarget_, payloadBuf, lenByte);
                                    // Dispatch to subscribers
                                    dispatchCallbacks(pktType, payloadBuf, lenByte);
                                    std::cout << "[TelProv] TARGET[" << static_cast<int>(lastTarget_.id)
                                              << "]: pos=(" << lastTarget_.x << "," << lastTarget_.y << ")"
                                              << std::endl;
                                }
                                break;
                            }
                            case dlink::PKT_AMMO: {
                                if (lenByte == sizeof(dlink::AmmoCfg)) {
                                    std::lock_guard<std::mutex> lock(mutex_);
                                    std::memcpy(&lastAmmo_, payloadBuf, lenByte);
                                    // Dispatch to subscribers
                                    dispatchCallbacks(pktType, payloadBuf, lenByte);
                                    std::cout << "[TelProv] AMMO: name=" << lastAmmo_.name
                                              << " mass=" << lastAmmo_.mass
                                              << " drag=" << lastAmmo_.drag
                                              << " lift=" << lastAmmo_.lift
                                              << " hitRadius=" << lastAmmo_.hitRadius
                                              << " nTargets=" << static_cast<int>(lastAmmo_.nTargets)
                                              << std::endl;
                                }
                                break;
                            }
                            case dlink::PKT_CONFIG: {
                                // DroneCfg received (optional, for additional config)
                                if (lenByte == sizeof(dlink::DroneCfg)) {
                                    dlink::DroneCfg cfg;
                                    std::memcpy(&cfg, payloadBuf, lenByte);
                                    // Dispatch to subscribers
                                    dispatchCallbacks(pktType, payloadBuf, lenByte);
                                    std::cout << "[TelProv] CONFIG: attackSpeed=" << cfg.attackSpeed
                                              << " accelPath=" << cfg.accelerationPath
                                              << " angularSpeed=" << cfg.angularSpeed
                                              << " timeScale=" << cfg.timeScale
                                              << std::endl;
                                }
                                break;
                            }
                            case dlink::PKT_RESULT: {
                                if (lenByte == sizeof(dlink::Result)) {
                                    dlink::Result result;
                                    std::memcpy(&result, payloadBuf, lenByte);
                                    std::cout << "[TelProv] RESULT: hit=" << static_cast<int>(result.hit)
                                              << " targetId=" << static_cast<int>(result.targetId)
                                              << " miss_m=" << result.miss_m
                                              << " drop_t_ms=" << result.drop_t_ms
                                              << std::endl;
                                }
                                break;
                            }
                            default:
                                std::cerr << "[TelProv] Unknown packet type: 0x"
                                          << std::hex << static_cast<int>(pktType) << std::dec
                                          << std::endl;
                                break;
                        }
                    }
                }
            }

            // Small sleep to avoid busy-waiting when no data
            if (n == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        std::cout << "[TelProv] Thread exiting." << std::endl;
    }

public:
    UartTelemetryProvider() = default;
    ~UartTelemetryProvider() override { stop(); }

    void setUartLink(IUartLink* uart) override { uart_ = uart; }
    void setGpioController(IDroneGpioController* gpio) override { gpio_ = gpio; }

    bool start() override {
        if (running_) return true;
        if (!uart_ || !uart_->isOpen()) {
            std::cerr << "[TelProv] UART not open" << std::endl;
            return false;
        }

        running_ = true;
        ready_ = false;
        simRunning_ = false;
        thread_ = std::thread(&UartTelemetryProvider::runLoop, this);
        return true;
    }

    void stop() override {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    bool isReady() const override { return ready_; }

    const dlink::Telemetry& getTelemetry() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastTel_;
    }

    const dlink::TargetPos& getTarget() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastTarget_;
    }

    const dlink::AmmoCfg& getAmmoConfig() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastAmmo_;
    }

    bool isSimulationRunning() const override { return simRunning_; }

    void registerCallback(dlink::PacketType type, PacketCallback cb) {
        callbacks_.emplace_back(type, cb);
    }

    // Static implementation for interface's global callback registration
    static void registerGlobalCallback(dlink::PacketType type, PacketCallback cb) {
        g_globalCallbacks.emplace_back(type, cb);
    }

    // Get reference to global callbacks (for external registration)
    static std::vector<CallbackEntry>& getGlobalCallbacks() {
        return g_globalCallbacks;
    }
};

// Define static member (use inner type CallbackEntry)
std::vector<UartTelemetryProvider::CallbackEntry> UartTelemetryProvider::g_globalCallbacks;

// Global callback registration helper (exposed for UART-based providers)
using GlobalPacketCallback = void(*)(const uint8_t*, uint8_t);
void uart_telemetry_register_global_callback(dlink::PacketType type, GlobalPacketCallback cb) {
    auto& gc = UartTelemetryProvider::getGlobalCallbacks();
    gc.emplace_back(type, cb);
}

// Factory function
std::unique_ptr<IUartTelemetryProvider> createUartTelemetryProvider() {
    return std::make_unique<UartTelemetryProvider>();
}

