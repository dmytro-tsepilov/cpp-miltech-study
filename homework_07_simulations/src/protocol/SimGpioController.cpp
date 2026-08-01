#include "protocol/IDroneGpioController.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <sys/stat.h>

// SimGpioController — simulation GPIO via debugfs (gpio-sim compatible)
class SimGpioController : public IDroneGpioController {
private:
    bool ready_ = false;
    int startLine_ = -1;
    int dropLine_ = -1;
    std::string chipName_;
    std::string gpioDir_;

    // Get the debugfs path for a GPIO line value file
    std::string getGpioPath(int line) {
        return gpioDir_ + "/line" + std::to_string(line) + "/value";
    }

    bool writeValue(int line, int value) {
        std::string path = getGpioPath(line);
        std::ofstream f(path.c_str(), std::ios::out);
        if (!f.is_open()) {
            // Try creating the directory first
            std::string dir = gpioDir_ + "/line" + std::to_string(line);
            mkdir(dir.c_str(), 0755);
            f.open(path.c_str(), std::ios::out);
            if (!f.is_open()) {
                std::cerr << "[SimGpio] Failed to write GPIO line " << line
                          << " to " << path << std::endl;
                return false;
            }
        }
        f << value;
        f.close();
        return true;
    }

    bool readValue(int line) {
        std::string path = getGpioPath(line);
        std::ifstream f(path.c_str(), std::ios::in);
        if (!f.is_open()) return false;
        int value = 0;
        f >> value;
        return value != 0;
    }

public:
    SimGpioController() = default;
    ~SimGpioController() override {
        // Не лишати лінії активними після виходу.
        if (ready_) {
            writeValue(dropLine_, 0);
            writeValue(startLine_, 0);
        }
    }

    bool init(const std::string& chipName, int startLine, int dropLine) override {
        chipName_ = chipName;
        startLine_ = startLine;
        dropLine_ = dropLine;

        gpioDir_ = "/tmp/gpio-sim/" + chipName;

        // Create directory if it doesn't exist
        mkdir(gpioDir_.c_str(), 0755);

        ready_ = true;

        // Базовий відомий стан: обидві лінії LOW (інакше DROP може
        // лишитись сталим HIGH від попереднього запуску).
        writeValue(startLine_, 0);
        writeValue(dropLine_, 0);

        std::cout << "[SimGpio] Initialized: chip=" << chipName
                  << " startLine=" << startLine
                  << " dropLine=" << dropLine << std::endl;
        return true;
    }

    void setStart(bool high) override {
        if (!ready_) return;
        writeValue(startLine_, high ? 1 : 0);
    }

    void pulseDrop(int durationMs = 80) override {
        if (!ready_) return;
        writeValue(dropLine_, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
        writeValue(dropLine_, 0);
    }

    bool isReady() const override { return ready_; }
};

// Factory function
std::unique_ptr<IDroneGpioController> createSimGpioController() {
    return std::make_unique<SimGpioController>();
}
