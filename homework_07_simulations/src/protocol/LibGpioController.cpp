#include "protocol/IDroneGpioController.h"
#include <memory>
#include <iostream>
#include <thread>
#include <chrono>

#if USE_GPIOD
#include <gpiod.h>

class LibGpioController : public IDroneGpioController {
private:
    bool ready_ = false;
    int startLineNum_ = -1;
    int dropLineNum_ = -1;
    std::string chipPath_;  // e.g. "/dev/gpiochip0"
    gpiod_chip* chip_ = nullptr;
    gpiod_line_request* request_ = nullptr;

public:
    LibGpioController() = default;
    ~LibGpioController() override {
        if (request_) {
            gpiod_line_request_release(request_);
        }
        if (chip_) {
            gpiod_chip_close(chip_);
        }
    }

    bool init(const std::string& chipName, int startLine, int dropLine) override {
        startLineNum_ = startLine;
        dropLineNum_ = dropLine;

        // Build the device path from chip name (e.g. "gpiochip0" -> "/dev/gpiochip0")
        chipPath_ = "/dev/" + chipName;

        // Open GPIO chip by path (v2.x batch API)
        chip_ = gpiod_chip_open(chipPath_.c_str());
        if (!chip_) {
            std::cerr << "[LibGpio] Failed to open GPIO chip: " << chipPath_ << std::endl;
            return false;
        }

        // Create line settings for output mode
        struct gpiod_line_settings* settings = gpiod_line_settings_new();
        if (!settings) {
            std::cerr << "[LibGpio] Failed to create line settings" << std::endl;
            gpiod_chip_close(chip_);
            return false;
        }

        // Set direction to output with initial value 0 for both lines
        if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_REQUEST_LINE_DIRECTION_OUTPUT) < 0) {
            std::cerr << "[LibGpio] Failed to set line direction" << std::endl;
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip_);
            return false;
        }

        if (gpiod_line_settings_set_output_value(settings, 0) < 0) {
            std::cerr << "[LibGpio] Failed to set output value" << std::endl;
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip_);
            return false;
        }

        // Create line config and add both lines (START and DROP)
        struct gpiod_line_config* lcfg = gpiod_line_config_new();
        if (!lcfg) {
            std::cerr << "[LibGpio] Failed to create line config" << std::endl;
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip_);
            return false;
        }

        unsigned int lines[] = { (unsigned int)startLineNum_, (unsigned int)dropLineNum_ };
        if (gpiod_line_config_add_line_settings(lcfg, settings, 2, lines) < 0) {
            std::cerr << "[LibGpio] Failed to add line settings to config" << std::endl;
            gpiod_line_config_free(lcfg);
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip_);
            return false;
        }

        // Request the lines from the chip
        request_ = gpiod_chip_request_lines(chip_, "drone", lcfg);
        if (!request_) {
            std::cerr << "[LibGpio] Failed to request GPIO lines" << std::endl;
            gpiod_line_config_free(lcfg);
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip_);
            return false;
        }

        // Set START line high immediately (ready signal to checker)
        if (gpiod_line_request_set_value(request_, startLineNum_, 1) < 0) {
            std::cerr << "[LibGpio] Failed to set START line high" << std::endl;
            gpiod_line_request_release(request_);
            gpiod_line_config_free(lcfg);
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip_);
            return false;
        }

        // Free config and settings (no longer needed after request)
        gpiod_line_config_free(lcfg);
        gpiod_line_settings_free(settings);

        ready_ = true;
        std::cout << "[LibGpio] Initialized: chip=" << chipName
                  << " startLine=" << startLineNum_
                  << " dropLine=" << dropLineNum_ << std::endl;
        return true;
    }

    void setStart(bool high) override {
        if (!ready_ || !request_) return;
        gpiod_line_request_set_value(request_, startLineNum_, high ? 1 : 0);
    }

    void pulseDrop(int durationMs = 80) override {
        if (!ready_ || !request_) return;
        gpiod_line_request_set_value(request_, dropLineNum_, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
        gpiod_line_request_set_value(request_, dropLineNum_, 0);
    }

    bool isReady() const override { return ready_; }
};

// Factory function for hardware mode
std::unique_ptr<IDroneGpioController> createLibGpioController() {
    return std::make_unique<LibGpioController>();
}

#else  // USE_GPIOD == 0 -- fallback GPIO controller

// When libgpiod is not available, use a no-op GPIO controller
class FallbackGpioController : public IDroneGpioController {
private:
    bool ready_ = false;
public:
    bool init(const std::string& chipName, int startLine, int dropLine) override {
        (void)chipName; (void)startLine; (void)dropLine;
        std::cerr << "[FallbackGpio] libgpiod not available, using sim GPIO" << std::endl;
        ready_ = true;
        return true;
    }
    void setStart(bool high) override { (void)high; }
    void pulseDrop(int durationMs = 80) override { (void)durationMs; }
    bool isReady() const override { return ready_; }
};

std::unique_ptr<IDroneGpioController> createLibGpioController() {
    std::cerr << "[LibGpio] USE_GPIOD not defined -- using fallback GPIO controller" << std::endl;
    return std::make_unique<FallbackGpioController>();
}

#endif  // USE_GPIOD
