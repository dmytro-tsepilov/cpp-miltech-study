#include <string>
#include <thread>
#include <chrono>

#include "common/macros.h"
#include "factories/ConfigLoaderFactory.h"
#include "factories/SolverFactory.h"
#include "factories/TargetProviderFactory.h"
#include "factories/ResultWriterFactory.h"
#include "mission/DronePhysics.h"
#include "mission/MissionProcessor.h"

int main(int argc, char** argv)
{
    // The executable expects folder path with simulation files
    if (argc < 2) {
        LOG("usage: <folder> - drone_simulations path to folder with simulation files (ammo.json, config.json, targets.json)\n");
        LOG("usage: --remote TEST_NUMBER - use input data from remote server\n");
        return 1;
    }

    std::string homeWork = "hw9";
    std::string testNumber = "5";
    bool remote = false;

    if (std::string(argv[1]) == "--remote") {
        if (argc != 3) {
            LOG("usage: --remote TEST_NUMBER - use input data from remote server\n");
            return 1;
        }
        remote = true;
        testNumber = argv[2]; 
    }

    std::string dataFolder = argv[1];

    std::unique_ptr<IConfigLoader> cfgLoader;
    std::unique_ptr<ITargetProvider> targetProvider;
    std::unique_ptr<IBallisticSolver> solver;

    SolverType solverType = SolverType::TABLE;

    if (solverType == SolverType::TABLE) {
        solver = createBallisticSolver(solverType, "homework_07_simulations/data/ballistic_table.txt");
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
        // Cast to HttpTargetProvider to access setTestName (not in base interface)
        // if (auto httpProvider = dynamic_cast<HttpTargetProvider*>(targetProvider.get())) {
        //     httpProvider->setTestName("test10_extreme");
        // }

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
