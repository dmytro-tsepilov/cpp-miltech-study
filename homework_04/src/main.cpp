#include <iostream>
#include <fstream>
#include <ostream>
#include <cmath>

struct NrkStep {
    long timestamp_ms;
    long fl_ticks;
    long fr_ticks;
    long bl_ticks;
    long br_ticks;
};

struct Coord {
    double x = 0;     // координати центру робота в метрах вiдносно точки старту;
    double y = 0;
    double theta = 0; //  орiєнтацiя (куди "дивиться" робот) у радiанах, вiдлiк вiд осi X:

    friend std::ostream& operator<<(std::ostream& os, const Coord& c) {
        os << "(x:" << c.x << "; y:" << c.y << "; theta:" << c.theta << ")";
        return os;
    }
};

struct SimStep {
    long timestamp_ms = 0;
    Coord pos;
};

const std::string DATA_FOLDER = "./homework_04/data/";

#define ENABLE_LOG	1
#define ENABLE_DEBUG  1

#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif


const int ticks_per_revolution = 1024;
const float wheel_radius_m = 0.3;
const float wheelbase_m = 1.0;

bool readDroneSteps(int &totalSteps, NrkStep **&nrkSteps, const std::string &filename);
bool saveResultsToJson(const SimStep* steps, const int &stepCount, const std::string &filename = "output.txt");

int main(int argc, char** argv) {
    // The program expects exactly one argument: a path to telemetry samples.
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    // TODO: implement wheel odometry for a 4-wheel differential-drive UGV.
    //
    // Model parameters:
    //   ticks_per_revolution = 1024
    //   wheel_radius_m       = 0.3
    //   wheelbase_m          = 1.0
    //
    // Input: a text file with 5 whitespace-separated values per line:
    //         timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
    // Output: a table on stdout, starting from the second sample:
    //         timestamp_ms x y theta

    //std::cout << "argv[1]: " << argv[1] << "\n";

    NrkStep** nrkSteps = nullptr;
    int totalSteps = 0;

    readDroneSteps(totalSteps, nrkSteps, argv[1]);

    LOG("Read file: '" << argv[1] << "' complete. Parsed " << totalSteps << " lines.");

    SimStep* steps = new SimStep[totalSteps];

    double distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;

    Coord currentPostion;

    for (int i = 1; i < totalSteps; i++)
    {
        // Step1 - calculate delta
        int d_fr = nrkSteps[i]->fr_ticks - nrkSteps[i-1]->fr_ticks;
        int d_fl = nrkSteps[i]->fl_ticks - nrkSteps[i-1]->fl_ticks;
        int d_bl = nrkSteps[i]->bl_ticks - nrkSteps[i-1]->bl_ticks;
        int d_br = nrkSteps[i]->br_ticks - nrkSteps[i-1]->br_ticks;

        // Step2 - 
        double d_left  = (d_fl + d_bl) / 2;
        double d_right = (d_fr + d_br) / 2;

        // Step3 - Conver impulse to meters
        double dL = d_left  * distance_per_tick;
        double dR = d_right * distance_per_tick;

        DEBUG("dL: " << dL << " dR: " << dR);

        // Step4 - calculate distance
        double distance = (dL + dR) / 2;          // пройдена вiдстань центру
        double dtheta = (dR - dL) / wheelbase_m;  // змiна орiєнтацiї

        DEBUG("step: " << i << " distance: " << distance << " dtheta: " << dtheta);

        // Step5 - identify drone position
        currentPostion.x += distance * cos(currentPostion.theta + dtheta / 2);
        currentPostion.y += distance * sin(currentPostion.theta + dtheta / 2);
        currentPostion.theta += dtheta;

        steps[i].timestamp_ms = nrkSteps[i]->timestamp_ms;
        steps[i].pos = currentPostion;

        DEBUG(currentPostion << " t: " << nrkSteps[i]->timestamp_ms);
    }

    // Save results to file
    saveResultsToJson(steps, totalSteps);

    // Free memory
    for (int i = 0; i < totalSteps; i++)
        delete[] nrkSteps[i];
    delete[] nrkSteps;
    nrkSteps = nullptr;

    // Free SimStep memory
    delete[] steps;
    steps = nullptr;

    return 0;
}

bool readDroneSteps(int &totalSteps, NrkStep **&nrkSteps, const std::string &filename)
{
    std::string line;

    std::ifstream inputFile(DATA_FOLDER + filename);
    if (!inputFile.is_open())
    {
        LOG("Error opening input file: " << DATA_FOLDER + filename);
        return false;
    }

    while (std::getline(inputFile, line)) {
        totalSteps++;
    }

    // Repoint file pointer to begin
    inputFile.clear();
    inputFile.seekg(0, std::ios::beg);

    nrkSteps = new NrkStep*[totalSteps];
    for (int i = 0; i < totalSteps; i++) {
        nrkSteps[i] = new NrkStep {};

        inputFile >> nrkSteps[i]->timestamp_ms >> 
            nrkSteps[i]->fl_ticks >> nrkSteps[i]->fr_ticks >>
            nrkSteps[i]->bl_ticks >> nrkSteps[i]->br_ticks;
    }

    inputFile.close();
    return true;
}

bool saveResultsToJson(const SimStep* steps, const int &stepCount, const std::string &filename)
{
    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        LOG("Error opening " << filename << " for writing");
        return false;
    }

    LOG("Writing " << stepCount << " steps to " << filename << " (TXT format)");

    for (int i = 0; i < stepCount; i++)
    {
        outFile << steps[i].timestamp_ms << " " << steps[i].pos.x << " " << steps[i].pos.y << " " << steps[i].pos.theta << std::endl;
    }

    outFile.close();

    LOG("Calculation completed: " << stepCount << " steps written to " << filename);

    return true;
}
