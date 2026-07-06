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
    std::string chipName_;
    gpiod_chip* chip_ = nullptr;
    gpiod_line* startLine_ = nullptr;
    gpiod_line* dropLine_ = nullptr;

public:
    LibGpioController() = default;
    ~LibGpioController() override {
        if (chip_) {
            gpiod_chip_close(chip_);
        }
    }

    bool init(const std::string& chipName, int startLine, int dropLine) override {
        chipName_ = chipName;
        startLineNum_ = startLine;
        dropLineNum_ = dropLine;

        chip_ = gpiod_chip_open_by_name(chipName_.c_str());
        if (!chip_) {
            std::cerr << "[LibGpio] Failed to open GPIO chip: " << chipName_ << std::endl;
            return false;
        }

        startLine_ = gpiod_chip_get_line(chip_, startLineNum_);
        if (!startLine_) {
            std::cerr << "[LibGpio] Failed to get START line: " << startLineNum_ << std::endl;
            gpiod_chip_close(chip_);
            return false;
        }

        dropLine_ = gpiod_chip_get_line(chip_, dropLineNum_);
        if (!dropLine_) {
            std::cerr << "[LibGpio] Failed to get DROP line: " << dropLineNum_ << std::endl;
            gpiod_chip_close(chip_);
            return false;
        }

        // Configure START and DROP lines as outputs, set high (ready signal)
        // gpiod_line_request_output returns 0 on success, or negative error code on failure.
        if (gpiod_line_request_output(startLine_, "drone", 0) < 0) {
            std::cerr << "[LibGpio] Failed to request START line as output" << std::endl;
            gpiod_chip_close(chip_);
            return false;
        }
        if (gpiod_line_request_output(dropLine_, "drone", 0) < 0) {
            std::cerr << "[LibGpio] Failed to request DROP line as output" << std::endl;
            gpiod_chip_close(chip_);
            return false;
        }

        // Set START high immediately (ready signal to checker)
        gpiod_line_set_value(startLine_, 1);

        ready_ = true;
        std::cout << "[LibGpio] Initialized: chip=" << chipName_
                  << " startLine=" << startLineNum_
                  << " dropLine=" << dropLineNum_ << std::endl;
        return true;
    }

    void setStart(bool high) override {
        if (!ready_ || !startLine_) return;
        gpiod_line_set_value(startLine_, high ? 1 : 0);
    }

    void pulseDrop(int durationMs = 80) override {
        if (!ready_ || !dropLine_) return;
        gpiod_line_set_value(dropLine_, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
        gpiod_line_set_value(dropLine_, 0);
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
