cmake_minimum_required(VERSION 3.8)
project(http_reporter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(perimeter_msgs REQUIRED)

include_directories(include)

# Find curl
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)

# HTTP reporter implementation library
add_library(http_reporter_lib src/http_reporter.cpp)
target_include_directories(http_reporter_lib PRIVATE include)
target_include_directories(http_reporter_lib PRIVATE ${CURL_INCLUDE_DIRS})
target_link_libraries(http_reporter_lib ${CURL_LIBRARIES})
ament_target_dependencies(http_reporter_lib rclcpp)

# HTTP reporter node
add_executable(reporter_node src/reporter_node.cpp)
ament_target_dependencies(reporter_node
  rclcpp
  std_msgs
  perimeter_msgs
)
target_link_libraries(reporter_node http_reporter_lib)

install(TARGETS
  reporter_node
  http_reporter_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Original test executable with HttpReporter implementation
  ament_add_gtest(test_http_reporter test/http_reporter_test.cpp src/http_reporter.cpp)
  target_include_directories(test_http_reporter PRIVATE include)
  ament_target_dependencies(test_http_reporter
    rclcpp
    perimeter_msgs
  )
  target_include_directories(test_http_reporter PRIVATE ${CURL_INCLUDE_DIRS})
  target_link_libraries(test_http_reporter ${CURL_LIBRARIES})
  
  # New serialization tests for HttpReporter
  ament_add_gtest(test_http_reporter_serialization
    test/http_reporter_serialization_test.cpp src/http_reporter.cpp
  )
  target_include_directories(test_http_reporter_serialization PRIVATE include)
  target_include_directories(test_http_reporter_serialization PRIVATE ${CURL_INCLUDE_DIRS})
  target_link_libraries(test_http_reporter_serialization ${CURL_LIBRARIES})
  
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
