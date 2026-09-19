cmake_minimum_required(VERSION 3.8)
project(mine_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(perimeter_msgs REQUIRED)

include_directories(include)

# Mine spawner node
add_executable(mine_spawner_node src/mine_spawner_node.cpp)
ament_target_dependencies(mine_spawner_node
  rclcpp
  std_msgs
  perimeter_msgs
)

install(TARGETS
  mine_spawner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config sdf urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mine_spawner test/mine_spawner_test.cpp)
  target_include_directories(test_mine_spawner PRIVATE include)
  ament_target_dependencies(test_mine_spawner
    rclcpp
    perimeter_msgs
  )
  
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
