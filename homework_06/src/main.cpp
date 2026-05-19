#include "ballistics.hpp"

#include <iostream>
#include <vector>
#include <cmath>

int main() {
    std::vector<std::string> lines;

    // Read input file
    if (!readInputFile(lines)) {
        std::cout << "Error reading file" << std::endl;
        return 1;
    }

    // Calculate only first line from file
    processCalculations((const char*)lines[0].c_str());

    return 0;
}
