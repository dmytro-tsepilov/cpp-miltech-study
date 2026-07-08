#include "config/UartConfigProvider.h"
#include "common/macros.h"
#include "protocol/IUartTelemetryProvider.h"
#include "protocol/drone_link.h"

#include <algorithm>
#include <cstring>

// Static member definitions
IUartTelemetryProvider* UartConfigProvider::g_uartTel = nullptr;
UartConfigProvider* UartConfigProvider::s_activeInstance = nullptr;

// Forward declaration of global callback registration (use plain function pointer)
using GlobalPacketCallback = void(*)(const uint8_t*, uint8_t);
extern void uart_telemetry_register_global_callback(dlink::PacketType type, GlobalPacketCallback cb);

// ---- Conversion helpers ----

void UartConfigProvider::convertDroneCfg(const dlink::DroneCfg& cfg)
{
    droneConfig_.attackSpeed = cfg.attackSpeed;
    droneConfig_.accelPath = cfg.accelerationPath;
    droneConfig_.angularSpeed = cfg.angularSpeed;
    droneConfig_.turnThreshold = cfg.turnThreshold;
    droneConfig_.timeScale = cfg.timeScale;

    // Крок симуляції з config чекера — важливий для нормування turnRate/accel.
    if (cfg.timeStep > 0.0f) {
        droneConfig_.simTimeStep = cfg.timeStep;
        droneConfig_.physicsTimeStep = cfg.timeStep;
    }

    // Fields not available from PKT_CONFIG — set defaults
    if (droneConfig_.startPos.x == 0 && droneConfig_.startPos.y == 0) {
        droneConfig_.startPos = {0, 0};
    }
    if (droneConfig_.altitude == 0) {
        droneConfig_.altitude = 100.0f; // default altitude
    }
    if (droneConfig_.initialDir == 0) {
        droneConfig_.initialDir = 0.0f;
    }
    if (droneConfig_.ammoName.empty()) {
        droneConfig_.ammoName = "unknown";
    }
    if (droneConfig_.arrayTimeStep == 0) {
        droneConfig_.arrayTimeStep = 0.05f;
    }
    if (droneConfig_.simTimeStep == 0) {
        droneConfig_.simTimeStep = 0.01f;
    }
    if (droneConfig_.hitRadius == 0) {
        droneConfig_.hitRadius = 5.0f;
    }
    if (droneConfig_.physicsTimeStep == 0) {
        droneConfig_.physicsTimeStep = 0.01f;
    }
}

void UartConfigProvider::convertAmmoCfg(const dlink::AmmoCfg& ammo)
{
    AmmoType a;
    // Copy name from fixed char array to std::string
    a.name = std::string(reinterpret_cast<const char*>(ammo.name), sizeof(ammo.name));
    // Trim trailing nulls/spaces
    a.name.erase(a.name.find_last_not_of(" \0") + 1);

    a.mass = ammo.mass;
    a.drag = ammo.drag;
    a.lift = ammo.lift;

    // hitRadius приходить саме з AMMO (а не з CONFIG).
    droneConfig_.hitRadius = ammo.hitRadius;

    // Store with lowercase key (same as FileConfigLoader)
    std::string lowerName(a.name);
    std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
    [](unsigned char c){ return static_cast<char>(std::tolower(c)); });

    ammoTypes_[lowerName] = a;
    droneConfig_.ammoName = lowerName;

    LOG("UartConfigProvider: AMMO loaded — name=" << lowerName
             << " mass=" << a.mass << " drag=" << a.drag << " lift=" << a.lift);
}

// ---- IConfigLoader implementation ----

bool UartConfigProvider::load()
{
    // Use global pointer if instance pointer not set
    auto* provider = uartTel_ ? uartTel_ : g_uartTel;
    if (!provider) {
        LOG("UartConfigProvider::load() — no UART telemetry provider set");
        return false;
    }

    // Wait until both CONFIG and AMMO packets have been received
    auto timeout = std::chrono::seconds(30);
    auto start = std::chrono::steady_clock::now();

    while (!loadComplete_.load()) {
        if (configReceived_.load() && ammoReceived_.load()) {
            loadComplete_ = true;
            // Початкова позиція/курс/висота — з реальної телеметрії чекера,
            // а не з дефолтів (PKT_CONFIG їх не містить).
            const auto& tel = provider->getTelemetry();
            droneConfig_.startPos = {tel.x, tel.y};
            droneConfig_.initialDir = tel.dir;
            if (tel.z > 0.0f) {
                droneConfig_.altitude = tel.z;
            }
            LOG("UartConfigProvider::load() — both CONFIG and AMMO received");
            return true;
        }

        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > timeout) {
            LOG("UartConfigProvider::load() — timeout waiting for packets");
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return true;
}

void UartConfigProvider::onConfigReceived(const dlink::DroneCfg& cfg)
{
    {
        std::lock_guard<std::mutex> lock(cvMutex_);
        convertDroneCfg(cfg);
    }
    configReceived_ = true;
    cv_.notify_all();

    LOG("UartConfigProvider: CONFIG received — attackSpeed=" << droneConfig_.attackSpeed
             << " accelPath=" << droneConfig_.accelPath << " angularSpeed=" << droneConfig_.angularSpeed);
}

void UartConfigProvider::onAmmoReceived(const dlink::AmmoCfg& ammo)
{
    {
        std::lock_guard<std::mutex> lock(cvMutex_);
        convertAmmoCfg(ammo);
    }
    ammoReceived_ = true;

    // If CONFIG was already received, both are now available
    if (configReceived_.load()) {
        loadComplete_ = true;
        cv_.notify_all();
    }
}

// ---- Static packet handlers ----

void UartConfigProvider::onConfigPacket(const uint8_t* payload, uint8_t len)
{
    if (len != sizeof(dlink::DroneCfg)) return;

    if (s_activeInstance) {
        dlink::DroneCfg cfg;
        std::memcpy(&cfg, payload, len);
        s_activeInstance->onConfigReceived(cfg);
    }
}

void UartConfigProvider::onAmmoPacket(const uint8_t* payload, uint8_t len)
{
    if (len != sizeof(dlink::AmmoCfg)) return;

    if (s_activeInstance) {
        dlink::AmmoCfg ammo;
        std::memcpy(&ammo, payload, len);
        s_activeInstance->onAmmoReceived(ammo);
    }
}

void UartConfigProvider::registerCallbacks()
{
    // Register global callbacks with the UART telemetry provider
    uart_telemetry_register_global_callback(dlink::PKT_CONFIG, &UartConfigProvider::onConfigPacket);
    uart_telemetry_register_global_callback(dlink::PKT_AMMO, &UartConfigProvider::onAmmoPacket);
}
