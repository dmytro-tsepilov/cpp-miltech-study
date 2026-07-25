cmake_minimum_required(VERSION 3.20)
project(mission_explorer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(mission_explorer_core
  src/map_builder.cpp
  src/explorer.cpp
)
target_include_directories(mission_explorer_core PUBLIC include)

ament_auto_add_executable(mission_explorer_node src/mission_explorer_node.cpp)
target_link_libraries(mission_explorer_node mission_explorer_core)

ament_auto_add_executable(payload_action_node src/payload_action_node.cpp)
ament_export_dependencies(rosidl_default_runtime)

install(TARGETS mission_explorer_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mission_explorer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS payload_action_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
