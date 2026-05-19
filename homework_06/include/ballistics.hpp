#include <strings.h>
#include <cstring>
#include <fstream>
#include <vector>

constexpr double g = 9.81;

bool readInputFile(std::vector<std::string>& lines, std::string filename = "input.txt");
bool writeOutputFile(const char* line, std::string filename = "output.txt");
int processCalculations(const char* line);
