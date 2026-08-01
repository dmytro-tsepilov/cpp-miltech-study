cmake_minimum_required(VERSION 3.20)
project(qos_profiles_demo)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(qos_demo_common INTERFACE)
target_include_directories(qos_demo_common INTERFACE include)
target_compile_features(qos_demo_common INTERFACE cxx_std_20)

add_executable(qos_counter_publisher src/qos_counter_publisher.cpp)
target_link_libraries(qos_counter_publisher qos_demo_common)
ament_target_dependencies(qos_counter_publisher rclcpp std_msgs)

add_executable(qos_counter_subscriber src/qos_counter_subscriber.cpp)
target_link_libraries(qos_counter_subscriber qos_demo_common)
ament_target_dependencies(qos_counter_subscriber rclcpp std_msgs)

install(TARGETS qos_counter_publisher qos_counter_subscriber DESTINATION lib/${PROJECT_NAME})

ament_package()
