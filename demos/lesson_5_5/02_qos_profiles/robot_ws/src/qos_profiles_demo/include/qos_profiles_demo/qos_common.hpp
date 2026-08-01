#pragma once

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace qos_profiles_demo {

inline std::string normalize(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

inline rclcpp::QoS make_qos(const std::string& reliability, const std::string& durability, const int depth)
{
  if (depth <= 0) {
    throw std::runtime_error("depth must be positive");
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(depth)));

  const auto normalized_reliability = normalize(reliability);
  if (normalized_reliability == "reliable") {
    qos.reliable();
  }
  else if (normalized_reliability == "best_effort") {
    qos.best_effort();
  }
  else {
    throw std::runtime_error("reliability must be reliable or best_effort");
  }

  const auto normalized_durability = normalize(durability);
  if (normalized_durability == "transient_local") {
    qos.transient_local();
  }
  else if (normalized_durability == "volatile") {
    qos.durability_volatile();
  }
  else {
    throw std::runtime_error("durability must be volatile or transient_local");
  }

  return qos;
}

inline void print_profile(
  const std::string& role, const std::string& topic, const std::string& reliability, const std::string& durability, const int depth)
{
  std::cout << "PROFILE" << " role=" << role << " topic=" << topic << " reliability=" << reliability << " durability=" << durability
            << " depth=" << depth << '\n';
}

}  // namespace qos_profiles_demo
