#include <string>

#include "common/macros.h"
#include "result/ResultWriter.h"
#include "drone/ConfigLoader.h"
#include "providers/TargetLoader.h"
#include "solvers/BallisticSolver.h"
#include "mission/MissionProcessor.h"

int main(int argc, char** argv)
{
    // The executable expects folder path with simulation files
    if (argc != 2) {
        LOG("usage: drone_simulations <input_path>\n");
        return 1;
    }

    std::string dataFolder = argv[1];

    auto solver = createBallisticSolver(SolverType::ANALYTICAL);
    auto targetProvider = createTargetProvider(SourceType::JSON, dataFolder.c_str());
    //auto targetProvider = createTargetProvider(SourceType::HTTP, "hw3", "0");
    // Cast to HttpTargetProvider to access setTestName (not in base interface)
    if (auto httpProvider = dynamic_cast<HttpTargetProvider*>(targetProvider.get())) {
        httpProvider->setTestName("test10_extreme");
    }

    auto cfgLoader = createConfigLoader(ConfigType::JSON, dataFolder.c_str());
    auto resultWriter = createResultWriter(DestType::JSON);

    auto mission = std::make_unique<MissionProcessor>(std::move(solver), std::move(targetProvider));
    mission->init(std::move(cfgLoader), std::move(resultWriter));

    while (mission->hasNext()) {
        mission->step();
    }

    mission->exportResults();

    return 0;
}
