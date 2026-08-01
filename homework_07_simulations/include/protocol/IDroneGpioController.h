#pragma once
// IDroneGpioController — GPIO abstraction for HW22 drone control (START/DROP lines)

#ifndef I_DRONE_GPIO_CONTROLLER_H
#define I_DRONE_GPIO_CONTROLLER_H

#include <memory>
#include <string>

class IDroneGpioController {
public:
    virtual ~IDroneGpioController() = default;

    // Initialize GPIO controller.
    // chipName: e.g. "gpiochip1" (sim) or "gpiochip0" (real Pi)
    // startLine: GPIO line number for START signal
    // dropLine: GPIO line number for DROP signal
    virtual bool init(const std::string& chipName, int startLine, int dropLine) = 0;

    // Set START line high (1) or low (0). Hold HIGH to signal readiness.
    virtual void setStart(bool high) = 0;

    // Generate a short DROP pulse (default 80ms). One-shot bomb release signal.
    virtual void pulseDrop(int durationMs = 80) = 0;

    // Check if initialization succeeded
    virtual bool isReady() const = 0;
};

// Factory function for hardware GPIO controller (libgpiod)
std::unique_ptr<IDroneGpioController> createLibGpioController();

#endif // I_DRONE_GPIO_CONTROLLER_H
