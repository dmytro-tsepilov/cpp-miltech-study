cmake_minimum_required(VERSION 3.8)
project(teleop_operator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Teleop driver library
add_library(teleop_driver_lib src/teleop_driver.cpp)
target_include_directories(teleop_driver_lib PRIVATE include)
ament_target_dependencies(teleop_driver_lib
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Teleop node
add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)
target_link_libraries(teleop_node teleop_driver_lib)

install(TARGETS
  teleop_node
  teleop_driver_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
