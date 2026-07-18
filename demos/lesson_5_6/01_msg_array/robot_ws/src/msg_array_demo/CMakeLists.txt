cmake_minimum_required(VERSION 3.20)
project(msg_array_demo)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_auto бере залежності з package.xml і прибирає зайвий шаблонний код.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# LocalScanLite містить масив CellObservationLite[], тому обидва .msg
# генеруються в одному виклику.
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CellObservationLite.msg"
  "msg/LocalScanLite.msg"
)

ament_auto_add_executable(scan_publisher src/scan_publisher.cpp)
ament_auto_add_executable(scan_subscriber src/scan_subscriber.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(scan_publisher "${cpp_typesupport_target}")
target_link_libraries(scan_subscriber "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package()

