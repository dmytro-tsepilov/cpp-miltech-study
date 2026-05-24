cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 04. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(drone_simulations CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## target_include_directories(json PUBLIC include)

# Один executable для задачі з drone simulations.
add_executable(drone_simulations src/main.cpp
    src/mission/MissionProcessor.cpp
    src/ballistic/AnalyticalSolver.cpp
    src/drone/ConfigLoader.cpp
    src/target/TargetLoader.cpp
    src/result/ResultWriter.cpp)

target_include_directories(drone_simulations PRIVATE include)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(drone_simulations PRIVATE -Wall -Wextra -pedantic)
