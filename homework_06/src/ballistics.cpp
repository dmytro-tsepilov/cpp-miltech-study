#include "ballistics.hpp"
#include <strings.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include "macros.h"

// Known ammo types lookup table

const int8_t BOMB_TYPES = 5;

const AmmoType bombTypes[BOMB_TYPES] = {{"VOG-17", 0.35f, 0.07f, 0.0f, false},
                                        {"M67", 0.6f, 0.10f, 0.0f, false},
                                        {"RKG-3", 1.2f, 0.10f, 0.0f, false},
                                        {"GLIDING-VOG", 0.45f, 0.10f, 1.0f, true},
                                        {"GLIDING-RKG", 1.4f, 0.10f, 1.0f, true}};

bool detectAmmoType(const char* ammoName, AmmoType& outAmmoType)
{
  for (size_t i = 0; i < sizeof(bombTypes) / sizeof(bombTypes[0]); ++i) {
    if (!strcasecmp(ammoName, bombTypes[i].name)) {
      outAmmoType = bombTypes[i];
      return true;
    }
  }
  outAmmoType = {};

  return false;
}

bool read_input_file(const std::string& filename, std::vector<std::string>& lines)
{
  lines.clear();
  std::ifstream inputFile(filename);
  if (!inputFile.is_open()) {
    return false;
  }

  std::string line;
  size_t totalSize = 0;
  while (std::getline(inputFile, line)) {
    size_t lineSize = line.size() + 1;
    if (totalSize + lineSize > 1024) {
      break;
    }
    totalSize += lineSize;
    lines.push_back(line);
  }
  return true;
}

// NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
bool write_output_file(const std::string& filename, const std::string& content)
{
  std::ofstream outputFile(filename);
  if (!outputFile.is_open()) {
    return false;
  }
  outputFile << content << std::endl;
  return true;
}

bool parse_input_line(const std::string& line, BallisticsInput& output)
{
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  static const std::regex pattern(
    R"(([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+(.+))");
  std::smatch match{};
  if (!std::regex_match(line, match, pattern)) {
    return false;
  }

  output.drone_x = std::stof(match[1].str());
  output.drone_y = std::stof(match[2].str());
  output.drone_z = std::stof(match[3].str());
  output.target_x = std::stof(match[4].str());
  output.target_y = std::stof(match[5].str());
  output.attack_speed = std::stof(match[6].str());
  output.acceleration_path = std::stof(match[7].str());
  output.ammo_name = match[8].str();
  return true;
}

DropSolution compute_drop_solution(const BallisticsInput& input, std::string& resultCalculation)
{
  DropSolution result{};
  result.valid = false;

  // Validate height
  if (input.drone_z <= 0) {
    result.error_message = "Height (drone_z) must be positive";
    return result;
  }

  // Validate attack speed
  if (input.attack_speed <= 0) {
    result.error_message = "Attack speed must be positive";
    return result;
  }

  // Look up ammo properties
  AmmoType ammo;

  if (!detectAmmoType(input.ammo_name.c_str(), ammo)) {
    result.error_message = "Unknown ammo type: " + input.ammo_name;
    return result;
  }

  // Validate ammo properties
  if (ammo.mass == 0.f) {
    result.error_message = "Invalid ammo properties (mass is zero)";
    return result;
  }

  // Calculate distance to target (horizontal)
  const double delta_x = input.target_x - input.drone_x;
  const double delta_y = input.target_y - input.drone_y;
  const double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

  if (distance <= 0) {
    result.error_message = "Target is at the same position as the drone";
    return result;
  }

  const double drag_coeff = ammo.drag;
  const double mass = ammo.mass;
  const double lift_coeff = ammo.lift;
  const double velocity = input.attack_speed;
  const double height = input.drone_z;

  // Calculate time of flight using Cardano's method
  const double t = calculateTimeToTarget(velocity, drag_coeff, mass, lift_coeff, height);

  if (t <= 0) {
    result.error_message = "No valid positive time solution";
    return result;
  }

  // Calculate horizontal distance traveled using the full expansion formula
  double horizontalDistance = calculateHorizontalDistance(input.attack_speed, ammo.drag, ammo.mass, ammo.lift, t);

  if (horizontalDistance <= 0) {
    result.error_message = "Calculated horizontal distance is zero or negative";
    return result;
  }

  char buf[64];
  resultCalculation = "";
  LOG("Horizontal distance to target: " << horizontalDistance);

  if (horizontalDistance + input.acceleration_path > distance) {
    LOG("Target is out of range");

    float xd_prime = input.target_x - (input.target_x - input.drone_x) * (horizontalDistance + input.acceleration_path) / distance;
    float yd_prime = input.target_y - (input.target_y - input.drone_y) * (horizontalDistance + input.acceleration_path) / distance;

    snprintf(buf, sizeof(buf), "%.3f %.3f ", xd_prime, yd_prime);
    resultCalculation += buf;
  }

  // Calculate targert coordinates after acceleration path
  float ratio = (distance - horizontalDistance) / distance;
  float fireX = input.drone_x + (input.target_x - input.drone_x) * ratio;
  float fireY = input.drone_y + (input.target_y - input.drone_y) * ratio;

  snprintf(buf, sizeof(buf), "%.3f %.3f", fireX, fireY);
  resultCalculation += buf;

  result.valid = true;
  LOG("Fire coordinates: (" << fireX << ", " << fireY << ")");

  return result;
}

double calculateHorizontalDistance(
  const float& attackSpeed, const float& ammoDrag, const float& ammoMass, const float& ammoLift, const double& time)
{
  // Calculate horizontal distance to target
  // h = V₀t − t²d·V₀/(2m) + t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
  //     + t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³)
  //     + t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)

  // h = V₀t − t²d·V₀/(2m) +
  double horizontalDistance = attackSpeed * time - pow(time, 2) * ammoDrag * attackSpeed / (2 * ammoMass) +
                              // t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
                              pow(time, 3) *
                                (6 * ammoDrag * g * ammoLift * ammoMass - 6 * pow(ammoDrag, 2) * (pow(ammoLift, 2) - 1) * attackSpeed) /
                                (36 * pow(ammoMass, 2)) +
                              // t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³) +
                              pow(time, 4) *
                                (-6 * pow(ammoDrag, 2) * g * ammoLift * (1 + pow(ammoLift, 2) + pow(ammoLift, 4)) * ammoMass +
                                 3 * pow(ammoDrag, 3) * pow(ammoLift, 2) * (1 + pow(ammoLift, 2)) * attackSpeed +
                                 6 * pow(ammoDrag, 3) * pow(ammoLift, 4) * (1 + pow(ammoLift, 2)) * attackSpeed) /
                                (36 * pow(1 + pow(ammoLift, 2), 2) * pow(ammoMass, 3)) +
                              // t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
                              pow(time, 5) *
                                (3 * pow(ammoDrag, 3) * g * pow(ammoLift, 3) * ammoMass -
                                 3 * pow(ammoDrag, 4) * pow(ammoLift, 2) * (1 + pow(ammoLift, 2)) * attackSpeed) /
                                (36 * (1 + pow(ammoLift, 2)) * pow(ammoMass, 4));

  return horizontalDistance;
}

double calculateTimeToTarget(const float& attackSpeed, const float& ammoDrag, const float& ammoMass, const float& ammoLift, const float& zd)
{
  // Calculate time to target using Cardano's method
  // These formulas match the correct implementation in main.cpp
  double a = ammoDrag * g * ammoMass - 2 * pow(ammoDrag, 2) * ammoLift * attackSpeed;
  double b = -3 * g * pow(ammoMass, 2) + 3 * ammoDrag * ammoLift * ammoMass * attackSpeed;
  double c = 6 * pow(ammoMass, 2) * zd;

  // Degenerate case: a ≈ 0 → use simple free fall formula
  if (std::abs(a) < 1e-12) {
    return std::sqrt(2 * zd / g);
  }

  // Calculate Cardano's method parameters (matching main.cpp formulas)
  double p = -pow(b, 2) / (3 * pow(a, 2));
  double q = (2 * pow(b, 3)) / (27 * pow(a, 3)) + c / a;

  // If p >= 0, use fallback formula
  if (p >= 0) {
    return std::sqrt(2 * zd / g);
  }

  double arg = 3 * q / (2 * p) * std::sqrt(-3 / p);

  // If arg outside [-1, 1], use fallback formula
  if (std::abs(arg) > 1) {
    return std::sqrt(2 * zd / g);
  }

  double phi = std::acos(arg);
  double t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * M_PI) / 3) - b / (3 * a);

  // If computed t is invalid, use fallback
  if (t <= 0 || !std::isfinite(t)) {
    return std::sqrt(2 * zd / g);
  }

  return t;
}
