cmake_minimum_required(VERSION 3.8)
project(perimeter_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MineDetection.msg"
  "msg/PerimeterStatus.msg"
  "msg/ClearanceReport.msg"
  "msg/MissionSummary.msg"
)

set(srv_files
  "srv/SwitchMode.srv"
  "srv/TriggerClearance.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
