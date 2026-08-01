cmake_minimum_required(VERSION 3.20)
project(iceoryx_compare_demo)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} msg/PayloadOneMb.msg msg/PayloadTenMb.msg
                           msg/PayloadHundredMb.msg)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ros2_payload_publisher src/ros2_loaned_payload_publisher.cpp)
target_link_libraries(ros2_payload_publisher "${cpp_typesupport_target}")
ament_target_dependencies(ros2_payload_publisher rclcpp)

add_executable(ros2_payload_subscriber src/ros2_loaned_payload_subscriber.cpp)
target_link_libraries(ros2_payload_subscriber "${cpp_typesupport_target}")
ament_target_dependencies(ros2_payload_subscriber rclcpp)

install(TARGETS ros2_payload_publisher ros2_payload_subscriber DESTINATION lib/${PROJECT_NAME})

# CycloneDDS і RouDi config-и встановлюються у share, але README запускає їх з src/, щоб під час
# лекції було видно, що саме змінюється.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
