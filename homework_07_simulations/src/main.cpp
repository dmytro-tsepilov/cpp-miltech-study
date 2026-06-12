#include <string>

#include "common/macros.h"
#include "factories/ConfigLoaderFactory.h"
#include "factories/SolverFactory.h"
#include "factories/TargetProviderFactory.h"
#include "factories/ResultWriterFactory.h"
#include "mission/MissionProcessor.h"

int main(int argc, char** argv)
{
    // The executable expects folder path with simulation files
    if (argc < 2) {
        LOG("usage: <folder> - drone_simulations path to folder with simulation files (ammo.json, config.json, targets.json)\n");
        LOG("usage: --remote TEST_NUMBER - use input data from remote server\n");
        return 1;
    }

    std::string homeWork = "hw3";
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
    auto solver = createBallisticSolver(SolverType::ANALYTICAL);
    if (remote) {
        targetProvider = createTargetProvider(SourceType::HTTP, homeWork, testNumber);
        // Cast to HttpTargetProvider to access setTestName (not in base interface)
        // if (auto httpProvider = dynamic_cast<HttpTargetProvider*>(targetProvider.get())) {
        //     httpProvider->setTestName("test10_extreme");
        // }

        cfgLoader = createConfigLoader(ConfigType::HTTP, homeWork, testNumber);
    }
    else {
        cfgLoader = createConfigLoader(ConfigType::JSON, dataFolder.c_str());
        targetProvider = createTargetProvider(SourceType::JSON, dataFolder.c_str());
    }

    auto resultWriter = createResultWriter(DestType::JSON);

    auto mission = std::make_unique<MissionProcessor>(std::move(solver), std::move(targetProvider));
    mission->init(std::move(cfgLoader), std::move(resultWriter));

    while (mission->hasNext()) {
        mission->step();
    }

    mission->exportResults();

    return 0;
}
