#pragma once

#include <string>
#include <vector>

constexpr double g = 9.81;

/**
 * Input structure for ballistics computation.
 * Represents drone position, target position, attack parameters, and ammo type.
 */
struct BallisticsInput {
  double drone_x;
  double drone_y;
  double drone_z;
  double target_x;
  double target_y;
  double attack_speed;
  double acceleration_path;
  std::string ammo_name;
};

/**
 * Solution structure for the drop point calculation.
 * Contains the computed fire coordinates and timing.
 */
struct DropSolution {
  double fire_x = 0.0;
  double fire_y = 0.0;
  double time_of_flight = 0.0;
  bool valid = false;
  std::string error_message;
};

/**
 * Ammo characteristics lookup table entry.
 */
struct AmmoType {
  const char *name;  // ammo type name
  float mass = 0;    // kg
  float drag = 0;    // drag coefficient
  float lift = 0;    // lift coefficient
  bool is_gliding;   // true if gliding ammo
};

/**
 * Compute the drop solution for a given ballistics input.
 *
 * Given drone position, target position, attack speed, acceleration path,
 * and ammo type, this function calculates the point from which the drone
 * should release the munition to hit the target.
 *
 * @param input The ballistics input parameters
 * @return DropSolution with fire coordinates and flight time, or invalid solution with error
 */
DropSolution compute_drop_solution(const BallisticsInput &input, std::string &resultCalculation);

/**
 * Look up ammo properties by name.
 *
 * @param ammo_name Name of the ammo type
 * @param out Output AmmoType if found
 * @return true if ammo type was found
 */
bool detectAmmoType(const char *ammoName, AmmoType &outAmmoType);

/**
 * Read input file and return lines of text.
 *
 * @param filename Path to the input file
 * @param lines Output vector of lines
 * @return true if file was read successfully
 */
bool read_input_file(const std::string &filename, std::vector<std::string> &lines);

/**
 * Write output to a file.
 *
 * @param filename Path to the output file
 * @param content Content to write
 * @return true if file was written successfully
 */
bool write_output_file(const std::string &filename, const std::string &content);

/**
 * Parse a single input line into BallisticsInput.
 *
 * Expected format: drone_x drone_y drone_z target_x target_y attack_speed
 *                 acceleration_path ammo_name
 *
 * @param line Input line string
 * @param output Parsed BallisticsInput
 * @return true if parsing was successful
 */
bool parse_input_line(const std::string &line, BallisticsInput &output);

double calculateHorizontalDistance(
  const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const double &time);
double calculateTimeToTarget(
  const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const float &zd);
