cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 04. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(drone_simulations CXX)

include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json
    GIT_TAG v3.12.0
)

# Для завдання з HttpTargetProvider нам потрібна бібліотека для HTTP-запитів.
FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib
    GIT_TAG v0.46.0
)

set(JSON_BuildTests OFF CACHE BOOL "Build tests" FORCE)
FetchContent_MakeAvailable(nlohmann_json)
FetchContent_MakeAvailable(httplib)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## target_include_directories(json PUBLIC include)

# Один executable для задачі з drone simulations.
file(GLOB_RECURSE SIMULATION_RESOURCES CONFIGURE_DEPENDS src/*.cpp)

add_executable(drone_simulations ${SIMULATION_RESOURCES})

target_link_libraries(drone_simulations PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(drone_simulations PRIVATE httplib::httplib)

target_include_directories(drone_simulations PRIVATE include)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(drone_simulations PRIVATE -Wall -Wextra -pedantic)
