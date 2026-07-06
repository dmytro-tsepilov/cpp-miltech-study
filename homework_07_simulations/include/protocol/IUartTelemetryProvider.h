#pragma once
// IUartTelemetryProvider — receives TELEMETRY/TARGET/AMMO packets from UART
// and provides them to the mission processor.

#ifndef I_UART_TELEMETRY_PROVIDER_H
#define I_UART_TELEMETRY_PROVIDER_H

#include <atomic>
#include <mutex>
#include "protocol/drone_link.h"

class IUartLink;
class IDroneGpioController;

class IUartTelemetryProvider {
public:
    virtual ~IUartTelemetryProvider() = default;

    // Attach UART link and GPIO controller
    virtual void setUartLink(IUartLink* uart) = 0;
    virtual void setGpioController(IDroneGpioController* gpio) = 0;

    // Start the background read thread. Blocks until checker sends first telemetry (after START=1).
    virtual bool start() = 0;

    // Stop the background read thread
    virtual void stop() = 0;

    // Check if provider is ready (has received at least one valid packet from checker)
    virtual bool isReady() const = 0;

    // Get last parsed telemetry snapshot
    virtual const dlink::Telemetry& getTelemetry() const = 0;

    // Get last parsed target position
    virtual const dlink::TargetPos& getTarget() const = 0;

    // Get last parsed ammo configuration
    virtual const dlink::AmmoCfg& getAmmoConfig() const = 0;

    // Check if checker has started simulation (START line was seen HIGH)
    virtual bool isSimulationRunning() const = 0;
};

#endif // I_UART_TELEMETRY_PROVIDER_H
