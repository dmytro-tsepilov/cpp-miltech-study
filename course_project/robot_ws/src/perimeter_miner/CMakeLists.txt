cmake_minimum_required(VERSION 3.8)
project(perimeter_miner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(perimeter_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Include directories
include_directories(include)

# Perimeter miner library (common components)
add_library(perimeter_miner_lib
  src/mode_switch.cpp
  src/lateral_pid.cpp
  src/hold_controller.cpp
  src/perimeter_loader.cpp
)
target_include_directories(perimeter_miner_lib PRIVATE include)
ament_target_dependencies(perimeter_miner_lib
  rclcpp
  perimeter_msgs
  yaml_cpp_vendor
)

# Perimeter miner node executable
add_executable(miner_node src/miner_node.cpp)
ament_target_dependencies(miner_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  perimeter_msgs
)
target_link_libraries(miner_node perimeter_miner_lib)

# Mode switch node executable
add_executable(mode_switch_node src/mode_switch_node.cpp)
ament_target_dependencies(mode_switch_node
  rclcpp
  std_msgs
  perimeter_msgs
)
target_link_libraries(mode_switch_node perimeter_miner_lib)

# Install executables
install(TARGETS
  miner_node
  mode_switch_node
  perimeter_miner_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Unit tests for perimeter tracker
  ament_add_gtest(test_perimeter_tracker
    test/perimeter_tracker_test.cpp
  )
  target_include_directories(test_perimeter_tracker PRIVATE include)
  ament_target_dependencies(test_perimeter_tracker
    rclcpp
    perimeter_msgs
  )
  target_link_libraries(test_perimeter_tracker perimeter_miner_lib)

  # Unit tests for mode switch
  ament_add_gtest(test_mode_switch
    test/mode_switch_test.cpp
  )
  target_include_directories(test_mode_switch PRIVATE include)
  ament_target_dependencies(test_mode_switch
    rclcpp
    perimeter_msgs
  )
  target_link_libraries(test_mode_switch perimeter_miner_lib)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
