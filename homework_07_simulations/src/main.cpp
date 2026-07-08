#include <string>
#include <thread>
#include <chrono>
#include <memory>
#include <iostream>
#include <cmath>

#include "common/macros.h"
#include "factories/ConfigLoaderFactory.h"
#include "factories/SolverFactory.h"
#include "factories/TargetProviderFactory.h"
#include "factories/ResultWriterFactory.h"
#include "mission/DronePhysics.h"
#include "mission/UartDroneState.h"
#include "mission/MissionProcessor.h"

// HW22: UART + GPIO includes
#include "protocol/drone_link.h"
#include "protocol/IDroneGpioController.h"
#include "protocol/IUartLink.h"
#include "protocol/IUartTelemetryProvider.h"
#include "protocol/IMissionCommandSource.h"

// UART-based providers for config and targets
#include "config/UartConfigProvider.h"
#include "providers/UartTargetProvider.h"
#include "providers/FixedTimeProvider.h"

// Forward declarations for factory functions
std::unique_ptr<IUartLink> createUartLink();
std::unique_ptr<IDroneGpioController> createSimGpioController();
std::unique_ptr<IUartTelemetryProvider> createUartTelemetryProvider();
std::unique_ptr<IMissionCommandSource> createMissionCommandSource();

// Helper function to parse command-line arguments.
// Supports both --key=value and --key value formats.
// Returns the value string if found, empty string otherwise.
std::string parseArgValue(int argc, char** argv, const std::string& key)
{
    for (int i = 1; i < argc; ++i) {
        // Check for --key=value format
        std::string arg = argv[i];
        if (arg.substr(0, key.size() + 2) == key + "=") {
            return arg.substr(key.size() + 3); // Skip "--key="
        }
        // Check for --key value format
        if (arg == key && i + 1 < argc) {
            return argv[i + 1];
        }
    }
    return "";
}

int main(int argc, char** argv)
{
    // The executable expects folder path with simulation files
    if (argc < 2) {
        LOG("usage: <folder> - drone_simulations path to folder with simulation files (ammo.json, config.json, targets.json)\n");
        LOG("usage: --remote TEST_NUMBER - use input data from remote server\n");
        LOG("usage: --btable BALLISTIC_TABLE_PATH - use input data from ballistic table file\n");
        LOG("\nHW22 UART + GPIO options:\n");
        LOG("  --uart DEVICE        - UART device path (e.g., /tmp/ttyA)\n");
        LOG("  --gpiochip NAME      - GPIO chip name (e.g., gpiochip1 for sim, gpiochip0 for real Pi)\n");
        LOG("  --start-line N       - GPIO line number for START signal\n");
        LOG("  --drop-line N        - GPIO line number for DROP signal\n");
        LOG("  --hw                 - Use real hardware mode (libgpiod)\n");
        return 1;
    }

    std::string homeWork = "hw9";
    std::string testNumber = "5";
    bool remote = false;
    std::string ballisticTablePath;

    // Parse --remote flag (supports --remote=test or --remote test)
    std::string remoteVal = parseArgValue(argc, argv, "--remote");
    if (!remoteVal.empty()) {
        remote = true;
        testNumber = remoteVal;
    }

    // Parse --btable flag (supports --btable=path or --btable /path)
    ballisticTablePath = parseArgValue(argc, argv, "--btable");
    if (ballisticTablePath.empty()) {
        ballisticTablePath = "homework_07_simulations/data/ballistic_table.txt"; // Default path
    }

    // Find the first non-flag argument as dataFolder
    std::string dataFolder;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.substr(0, 2) != "--") {
            dataFolder = arg;
            break;
        }
    }

    // HW22: Parse UART and GPIO arguments
    std::string uartDevice = parseArgValue(argc, argv, "--uart");
    std::string gpioChipName = parseArgValue(argc, argv, "--gpiochip");
    std::string startLineStr = parseArgValue(argc, argv, "--start-line");
    std::string dropLineStr = parseArgValue(argc, argv, "--drop-line");

    // Parse --hw flag: supports bare "--hw" or "--hw=value"
    bool hwMode = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--hw") {
            hwMode = true;
            break;
        }
        if (arg.substr(0, 5) == "--hw=") {
            hwMode = true;
            break;
        }
    }

    // Set defaults based on hardware mode
    int startLine = 24;   // sim default
    int dropLine = 23;    // sim default
    std::string defaultUart = "/tmp/ttyA";  // sim default
    std::string defaultGpioChip = "gpiochip1";  // sim default

    if (hwMode) {
        // Hardware defaults (Raspberry Pi)
        defaultUart = "/dev/ttyAMA1";
        defaultGpioChip = "gpiochip0";
    }

    int startLineNum = startLineStr.empty() ? startLine : std::stoi(startLineStr);
    int dropLineNum = dropLineStr.empty() ? dropLine : std::stoi(dropLineStr);

    // Default UART device if not specified
    if (uartDevice.empty()) {
        uartDevice = defaultUart;
    }

    // Default GPIO chip name if not specified
    if (gpioChipName.empty()) {
        gpioChipName = defaultGpioChip;
    }

    // HW22 UART mode is selected when a UART device is passed (or --hw is used);
    // otherwise fall back to the legacy file/HTTP simulation below.
    bool uartMode = !parseArgValue(argc, argv, "--uart").empty() || hwMode;

    // ==========================================
    // HW22: UART + GPIO mode
    // ==========================================
    if (uartMode) {
        LOG("=== HW22 UART + GPIO Mode ===");
        LOG("Mode: " << (hwMode ? "HARDWARE" : "SIMULATION"));
        LOG("UART device: " << uartDevice);
        LOG("GPIO chip: " << gpioChipName << " START=" << startLineNum << " DROP=" << dropLineNum);

        // 1. Create and initialize GPIO controller (sim or hardware based on --hw flag)
        std::unique_ptr<IDroneGpioController> gpio;
        if (hwMode) {
            gpio = createLibGpioController();
        } else {
            gpio = createSimGpioController();
        }
        if (!gpio->init(gpioChipName, startLine, dropLine)) {
            LOG("Failed to initialize GPIO controller");
            return 1;
        }

        // 2. Open UART link
        auto uart = createUartLink();
        if (!uart->open(uartDevice)) {
            LOG("Failed to open UART device: " << uartDevice);
            return 1;
        }
        LOG("UART opened: " << uartDevice);

        // Set START high immediately (ready signal to checker)
        gpio->setStart(true);
        LOG("START line set HIGH — waiting for checker to start simulation");
        //gpio->setStart(false); // Set low after signaling ready

        // 3. Create and wire UART-based providers BEFORE starting telemetry
        auto telProvider = createUartTelemetryProvider();
        telProvider->setUartLink(uart.get());
        telProvider->setGpioController(gpio.get());

        // Wire global pointers FIRST (needed by callback static instances)
        UartConfigProvider::setGlobalUartTelemetryProvider(telProvider.get());
        UartTargetProvider::setGlobalUartTelemetryProvider(telProvider.get());

        // Register callbacks BEFORE starting the thread so no packets are missed
        UartConfigProvider::registerCallbacks();
        UartTargetProvider::registerCallbacks();

        // Create UART-backed config and target providers EARLY
        // and set instance pointers BEFORE starting telemetry thread.
        // This ensures packets arriving immediately after thread start
        // are dispatched to the correct instances.
        auto uartCfg = std::make_unique<UartConfigProvider>();
        UartConfigProvider::setInstance(uartCfg.get());

        auto uartTgt = std::make_unique<UartTargetProvider>();
        UartTargetProvider::setInstance(uartTgt.get());

        if (!telProvider->start()) {
            LOG("Failed to start telemetry provider");
            uart->close();
            return 1;
        }
        LOG("Telemetry provider started, waiting for first packet...");

        // Wait for checker to send first telemetry (blocks until START=1 is seen)
        int waitCount = 0;
        while (!telProvider->isReady() && waitCount < 5000) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            waitCount++;
        }

        if (!telProvider->isReady()) {
            LOG("Timeout waiting for telemetry from checker");
            telProvider->stop();
            uart->close();
            return 1;
        }
        LOG("First telemetry received, simulation is running!");

        // Get initial AMMO config from checker
        const auto& ammoCfg = telProvider->getAmmoConfig();
        LOG("AMMO received: name=" << ammoCfg.name << " mass=" << ammoCfg.mass
                                   << " drag=" << ammoCfg.drag << " lift=" << ammoCfg.lift);

        // 4. Create UART-based config and target providers
        std::unique_ptr<IConfigLoader> cfgLoader;
        std::unique_ptr<ITargetProvider> targetProvider;
        std::unique_ptr<IBallisticSolver> solver;

        // UART-режим: аналітичний солвер — не потребує зовнішнього файлу
        // таблиці, якого немає на платі (deploy.sh синхронізує лише src/ та include/).
        solver = createBallisticSolver(SolverType::ANALYTICAL);

        if (solver == nullptr) {
            LOG("Failed to create ballistic solver");
            telProvider->stop();
            uart->close();
            return 1;
        }

        // cfgLoader and targetProvider are already created above (before telemetry thread start).
        // Wrap them in smart pointers for ownership transfer to mission init.
        cfgLoader = std::unique_ptr<IConfigLoader>(uartCfg.release());
        targetProvider = std::unique_ptr<ITargetProvider>(uartTgt.release());

        auto resultWriter = createResultWriter(DestType::JSON);
        if (resultWriter == nullptr) {
            LOG("Failed to create result writer");
            telProvider->stop();
            targetProvider.reset();
            cfgLoader.reset();
            uart->close();
            return 1;
        }

        // UART-режим: стан дрона надходить від чекера, тож замість локальної
        // фізики використовуємо UartDroneState (місія наводиться по телеметрії).
        auto droneState = std::make_unique<UartDroneState>();
        UartDroneState* dronePtr = droneState.get();

        auto mission = std::make_unique<MissionProcessor>(std::move(solver), std::move(targetProvider));
        if (!mission->init(std::move(cfgLoader), std::move(resultWriter), dronePtr)) {
            LOG("Failed to initialize mission");
            telProvider->stop();
            droneState.reset();
            uart->close();
            return 1;
        }

        LOG("Mission ready");

        // 5. Create drone control module (converts mission decision -> UART CONTROL).
        auto cmdSource = createMissionCommandSource();

        // Control-normalization scales computed by the mission from the checker config.
        const double maxTurnPerStep = mission->getMaxTurnPerStep();
        const float  accelPerStep   = mission->getAccelPerStep();
        LOG("Control module ready: maxTurnPerStep=" << maxTurnPerStep
                                                    << " accelPerStep=" << accelPerStep);

        // 6. Main control loop
        LOG("=== Starting main control loop ===");

        int step = 0;
        const int maxSteps = 10000;

        while (step < maxSteps) {
            if (!telProvider->isSimulationRunning()) {
                LOG("Simulation not yet running (waiting for START)...");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Feed checker telemetry into the mission's drone-state source.
            const auto& t = telProvider->getTelemetry();
            DroneTelemetry dt;
            dt.pos = {t.x, t.y};
            dt.direction = t.dir;
            dt.speed = t.speed;
            dt.state = t.state;
            dt.timeSecSinceStart = t.t_ms / 1000.0f;
            dronePtr->setTelemetry(dt);

            // Run one guidance step: mission computes desired heading/speed + drop decision.
            mission->step();

            // Drop decision comes from the mission (ballistic fire point reached).
            if (mission->shouldDrop()) {
                LOG("*** DROP PULSE TRIGGERED at step " << step << " ***");
                uart->sendControl(0.0f, 0.0f);
                gpio->pulseDrop(180);  // 80ms pulse
                break;
            }

            // No valid firing solution / mission ended without a drop.
            if (!mission->hasNext()) {
                LOG("Mission ended without drop at step " << step);
                break;
            }

            // Convert the mission decision into a normalized CONTROL command.
            DroneCommand cmd = dronePtr->getLastCommand();
            float accel = 0.0f;
            float turnRate = 0.0f;
            cmdSource->computeControl(cmd, dt, maxTurnPerStep, accelPerStep, accel, turnRate);

            if (!uart->sendControl(accel, turnRate)) {
                LOG("Failed to send CONTROL command at step " << step);
            }

            if (step % 100 == 0) {
                LOG("Step " << step << ": tel_pos=(" << t.x << "," << t.y
                           << ") speed=" << t.speed << " dir=" << t.dir
                           << ") cmd=[accel=" << accel << " turn=" << turnRate << "]");
            }

            // Pace the loop at the mission time step.
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            step++;
        }

        // 7. Export results
        mission->exportResults();
        LOG("Results exported.");

        // 8. Cleanup
        gpio->setStart(false);   // сказати чекеру «завершено» і не лишати START у HIGH
        telProvider->stop();
        uart->close();

        LOG("=== HW22 session complete ===");
        return 0;
    }

    // ==========================================
    // Legacy: File-based simulation mode (original)
    // ==========================================
    std::unique_ptr<IConfigLoader> cfgLoader;
    std::unique_ptr<ITargetProvider> targetProvider;
    std::unique_ptr<IBallisticSolver> solver;

    SolverType solverType = SolverType::TABLE;

    if (solverType == SolverType::TABLE) {
        solver = createBallisticSolver(solverType, ballisticTablePath);
    } else {
        solver = createBallisticSolver(solverType);
    }

    if (solver == nullptr) {
        LOG("Failed to create ballistic solver");
        return 1;
    }

    if (remote) {
        targetProvider = createTargetProvider(SourceType::HTTP, homeWork, testNumber);
        if (targetProvider == nullptr) {
            LOG("Failed to create HTTP target provider");
            return 1;
        }

        cfgLoader = createConfigLoader(ConfigType::HTTP, homeWork, testNumber);
        if (cfgLoader == nullptr) {
            LOG("Failed to create HTTP config loader");
            return 1;
        }
    }
    else {
        cfgLoader = createConfigLoader(ConfigType::JSON, dataFolder.c_str());
        if (cfgLoader == nullptr) {
            LOG("Failed to create JSON config loader");
            return 1;
        }
        targetProvider = createTargetProvider(SourceType::JSON, dataFolder.c_str());
        if (targetProvider == nullptr) {
            LOG("Failed to create JSON target provider");
            return 1;
        }
    }

    auto resultWriter = createResultWriter(DestType::JSON);
    if (resultWriter == nullptr) {
        LOG("Failed to create result writer");
        return 1;
    }

    auto physics = std::make_unique<DronePhysics>();
    DronePhysics* physicsPtr = physics.get();
    ITargetProvider* providerPtr = targetProvider.get();

    auto mission = std::make_unique<MissionProcessor>(std::move(solver), std::move(targetProvider));
    if (!mission->init(std::move(cfgLoader), std::move(resultWriter), physicsPtr)) {
        LOG("Failed to initialize mission");
        return 1;
    }
    MissionProcessor* missionPtr = mission.get();

    // Three separate threads: target provider, drone physics, mission processor
    std::thread providerThread(&ITargetProvider::run, providerPtr);
    std::thread physicsThread(&DronePhysics::run, physicsPtr);
    std::thread missionThread(&MissionProcessor::run, missionPtr);

    // Wait until all threads are ready to start
    while (!providerPtr->isThreadReady() || !physicsPtr->isThreadReady() || !missionPtr->isThreadReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Synchronized start — targets do not move until the rest of the system is ready
    providerPtr->start();
    physicsPtr->start();
    missionPtr->start();

    // Mission is the only thread we wait for to finish
    missionThread.join();

    // Stop physics and target provider using stop flags
    physicsPtr->stop();
    providerPtr->stop();
    providerThread.join();
    physicsThread.join();

    mission->exportResults();

    return 0;
}
