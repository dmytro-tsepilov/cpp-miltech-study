cmake_minimum_required(VERSION 3.20)

# Project for homework 16: I2C ADS1115 sensor reader.
project(i2c_ads1115 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SIMULATION_RESOURCES CONFIGURE_DEPENDS src/*.cpp)

add_executable(i2c_test ${SIMULATION_RESOURCES})
target_include_directories(i2c_test PRIVATE include)

target_include_directories(i2c_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(i2c_test PRIVATE -Wall -Wextra -pedantic)
