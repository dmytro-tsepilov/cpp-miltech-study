cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 05. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(ballistics_check CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ballistics src/ballistics.cpp)
target_include_directories(ballistics PUBLIC include)

# Один executable для задачі з ballistics check.
add_executable(ballistics_check src/main.cpp)
target_link_libraries(ballistics_check PRIVATE ballistics)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(ballistics_check PRIVATE -Wall -Wextra -pedantic)