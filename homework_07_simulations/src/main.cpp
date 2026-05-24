#include <string>

#include "common/macros.h"
#include "result/ResultWriter.h"
#include "drone/ConfigLoader.h"
#include "target/TargetLoader.h"
#include "ballistic/BallisticSolver.h"
#include "mission/MissionProcessor.h"

int main(int argc, char** argv)
{
    // The executable expects folder path with simulation files
    if (argc != 2) {
        LOG("usage: drone_simulations <input_path>\n");
        return 1;
    }

    std::string dataFolder = argv[1];

    IBallisticSolver *solver = createBallisticSolver(SolverType::ANALYTICAL);
    ITargetProvider *targetProvider = createTargetProvider(SourceType::JSON, dataFolder.c_str());
    FileConfigLoader *cfgLoader = new FileConfigLoader();
    IResultWriter *wrtierProvider = createResultWriter(DestType::JSON);

    MissionProcessor *mission = new MissionProcessor(solver, targetProvider);
    cfgLoader->setFolderPath(dataFolder);
    mission->init(cfgLoader, wrtierProvider);

    mission->calculateFlow();

    return 0;
}
