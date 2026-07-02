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

set(JSON_BuildTests OFF CACHE BOOL "Build tests" FORCE)
FetchContent_MakeAvailable(nlohmann_json)

# Опція для включення/виключення HTTP-функціоналу
# cmake --preset debug -DENABLE_HTTP=ON
# cmake --preset debug -DENABLE_HTTP=OFF
if(NOT DEFINED ENABLE_HTTP)
    option(ENABLE_HTTP "Enable HTTP-based config and target providers" ON)
endif()

# Для завдання з HttpTargetProvider нам потрібна бібліотека для HTTP-запитів.
if(ENABLE_HTTP)
FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib
    GIT_TAG v0.47.0
)

set(CPPHTTPLIB_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(httplib)
endif()

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## target_include_directories(json PUBLIC include)

# Один executable для задачі з drone simulations.
file(GLOB_RECURSE SIMULATION_RESOURCES CONFIGURE_DEPENDS src/*.cpp)

# Включаємо HTTP-файли тільки якщо ENABLE_HTTP увімкнено
if(NOT ENABLE_HTTP)
    list(FILTER SIMULATION_RESOURCES EXCLUDE REGEX "HttpConfigLoader\\.cpp|HttpTargetProvider\\.cpp")
endif()

add_executable(drone_simulations ${SIMULATION_RESOURCES})

target_link_libraries(drone_simulations PRIVATE nlohmann_json::nlohmann_json)
if(ENABLE_HTTP)
target_link_libraries(drone_simulations PRIVATE httplib::httplib)
endif()

target_include_directories(drone_simulations PRIVATE include)

if(ENABLE_HTTP)
target_compile_definitions(drone_simulations PRIVATE ENABLE_HTTP=1)
else()
target_compile_definitions(drone_simulations PRIVATE ENABLE_HTTP=0)
endif()

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(drone_simulations PRIVATE -Wall -Wextra -pedantic)
