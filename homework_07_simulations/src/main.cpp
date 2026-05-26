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
    //ITargetProvider *targetProvider = createTargetProvider(SourceType::JSON, dataFolder.c_str());
    ITargetProvider *targetProvider = createTargetProvider(SourceType::HTTP, "http://cppmiltech.com.ua", "/hw3/api/tests/test10_extreme");
    
    IConfigLoader *cfgLoader = createConfigLoader(ConfigType::JSON, dataFolder.c_str());
    IResultWriter *wrtierProvider = createResultWriter(DestType::JSON);

    MissionProcessor *mission = new MissionProcessor(solver, targetProvider);
    mission->init(cfgLoader, wrtierProvider);

    while (mission->hasNext()) {
        mission->step();
    }

    mission->exportResults();

    delete solver;
    delete targetProvider;
    delete cfgLoader;
    delete wrtierProvider;
    delete mission;

    return 0;
}
