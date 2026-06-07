#include "ballistics.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "macros.h"

#define ENABLE_LOG 1
#define ENABLE_DEBUG 1

int main(int argc, char* argv[])
{
  std::string input_file = "data/input.txt";
  std::string output_file = "output.txt";

  if (argc > 1) {
    input_file = std::string(argv[1]);
  }
  if (argc > 2) {
    output_file = std::string(argv[2]);
  }

  std::vector<std::string> lines;
  if (!read_input_file(input_file, lines)) {
    LOG("Error: cannot read input file: " << input_file);
    return 1;
  }

  if (lines.empty()) {
    LOG("Error: input file is empty: " << input_file);
    return 1;
  }

  std::string output_content;
  std::string resultCalculation;

  BallisticsInput input;
  if (!parse_input_line(lines[0], input)) {
    LOG("Warning: cannot parse line skipping: " << lines[0]);
    return 1;
  }

  DropSolution result = compute_drop_solution(input, resultCalculation);

  if (!result.valid) {
    LOG("Error on line: " << result.error_message);
    return 1;
  }

  if (!write_output_file(output_file, resultCalculation)) {
    LOG("Error: cannot write output file: " << output_file);
    return 1;
  }

  LOG("Results written to " << output_file);
  return 0;
}
