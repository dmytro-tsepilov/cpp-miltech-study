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
#include "mission/MissionProcessor.h"

// HW22: UART + GPIO includes
#include "protocol/drone_link.h"
#include "protocol/IDroneGpioController.h"
#include "protocol/IUartLink.h"
#include "protocol/IUartTelemetryProvider.h"
#include "protocol/IMissionCommandSource.h"

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
        startLine = 27;
        dropLine = 22;
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

    // ==========================================
    // HW22: UART + GPIO mode (default)
    // ==========================================
    {
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

        // Set START high immediately (ready signal to checker)
        gpio->setStart(true);
        LOG("START line set HIGH — waiting for checker to start simulation");

        // 2. Open UART link
        auto uart = createUartLink();
        if (!uart->open(uartDevice)) {
            LOG("Failed to open UART device: " << uartDevice);
            return 1;
        }
        LOG("UART opened: " << uartDevice);

        // 3. Create and start telemetry provider (background thread)
        auto telProvider = createUartTelemetryProvider();
        telProvider->setUartLink(uart.get());
        telProvider->setGpioController(gpio.get());

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

        // 4. Create traditional simulation components for ballistic calculations
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
            telProvider->stop();
            uart->close();
            return 1;
        }

        // Use local config loader for ballistic parameters
        cfgLoader = createConfigLoader(ConfigType::JSON, dataFolder.c_str());
        if (cfgLoader == nullptr) {
            LOG("Failed to create JSON config loader");
            telProvider->stop();
            uart->close();
            return 1;
        }

        targetProvider = createTargetProvider(SourceType::JSON, dataFolder.c_str());
        if (targetProvider == nullptr) {
            LOG("Failed to create JSON target provider");
            telProvider->stop();
            cfgLoader.reset();
            uart->close();
            return 1;
        }

        auto resultWriter = createResultWriter(DestType::JSON);
        if (resultWriter == nullptr) {
            LOG("Failed to create result writer");
            telProvider->stop();
            targetProvider.reset();
            cfgLoader.reset();
            uart->close();
            return 1;
        }

        auto physics = std::make_unique<DronePhysics>();
        DronePhysics* physicsPtr = physics.get();

        auto mission = std::make_unique<MissionProcessor>(std::move(solver), std::move(targetProvider));
        if (!mission->init(std::move(cfgLoader), std::move(resultWriter), physicsPtr)) {
            LOG("Failed to initialize mission");
            telProvider->stop();
            physics.reset();
            uart->close();
            return 1;
        }
        // Note: missionPtr not needed — we use smart pointer directly

        // 5. Create mission command source for CONTROL generation
        auto cmdSource = createMissionCommandSource();
        cmdSource->init(ammoCfg.hitRadius > 0 ? 50.0f : 30.0f, 1.5f); // Will be updated from config

        // Get actual drone config for proper parameters
        if (cfgLoader->load()) {
            auto droneConfig = cfgLoader->getConfig();
            cmdSource->init(droneConfig.attackSpeed, droneConfig.angularSpeed);
            LOG("Mission command source initialized: attackSpeed=" << droneConfig.attackSpeed
                                                                    << " angularSpeed=" << droneConfig.angularSpeed);
        }

        // 6. Initialize physics with start position from config
        if (cfgLoader->load()) {
            auto droneConfig = cfgLoader->getConfig();
            physics->init(droneConfig.startPos, droneConfig.initialDir,
                         droneConfig.physicsTimeStep, droneConfig.timeScale);
            LOG("Physics initialized: startPos=" << droneConfig.startPos.x << "," << droneConfig.startPos.y
                                                  << " initialDir=" << droneConfig.initialDir);
        }

        // 7. Main control loop
        LOG("=== Starting main control loop ===");

        int step = 0;
        const int maxSteps = 10000;

        while (step < maxSteps) {
            // Get latest telemetry from UART
            const auto& tel = telProvider->getTelemetry();
            const auto& target = telProvider->getTarget();

            if (!telProvider->isSimulationRunning()) {
                LOG("Simulation not yet running (waiting for START)...");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Check if mission is complete
            if (!mission->hasNext()) {
                LOG("Mission complete at step " << step);
                break;
            }

            // Update mission processor with current telemetry
            // (MissionProcessor reads from physics internally)

            // Generate CONTROL command from mission logic
            float accel = 0.0f;
            float turnRate = 0.0f;
            cmdSource->generateCommand(tel, target, accel, turnRate);

            // Send CONTROL via UART
            if (!uart->sendControl(accel, turnRate)) {
                LOG("Failed to send CONTROL command at step " << step);
            }

            // Check if should drop bomb
            if (cmdSource->shouldDrop()) {
                LOG("*** DROP PULSE TRIGGERED at step " << step << " ***");
                gpio->pulseDrop(80);  // 80ms pulse
                cmdSource->resetDrop();
            }

            // Sleep for simulation timestep (scaled by timeScale)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (step % 100 == 0) {
                LOG("Step " << step << ": tel_pos=(" << tel.x << "," << tel.y
                           << ") speed=" << tel.speed << " dir=" << tel.dir
                           << " target=(" << target.x << "," << target.y
                           << ") cmd=[accel=" << accel << " turn=" << turnRate << "]");
            }

            step++;
        }

        // 8. Export results
        mission->exportResults();
        LOG("Results exported.");

        // 9. Cleanup
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
