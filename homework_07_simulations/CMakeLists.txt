cmake_minimum_required(VERSION 3.20)

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
    GIT_TAG v0.49.0
)

set(CPPHTTPLIB_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(httplib)
endif()

# HW22: GPIO support via libgpiod (for real Raspberry Pi hardware)
# This is optional — if not available, simulation mode uses debugfs instead.
option(ENABLE_GPIO "Enable GPIO support via libgpiod" ON)

# Clear cached GPIO_AVAILABLE before re-detecting to avoid stale values from previous configure runs.
# This is critical for cross-compilation: a previous x86 configure may have found libgpiod,
# but the target architecture (e.g., aarch64) may not have it installed.
unset(GPIO_AVAILABLE CACHE)
unset(GPIOD_INCLUDE_DIR CACHE)
unset(GPIOD_LIBRARIES CACHE)
set(GPIO_AVAILABLE OFF CACHE BOOL "Whether GPIO/libgpiod is available")

if(ENABLE_GPIO)
    # Try to find libgpiod headers and libraries
    find_path(GPIOD_INCLUDE_DIR gpiod.h)
    find_library(GPIOD_LIBRARIES NAMES gpiod gpiocore)

    if(GPIOD_INCLUDE_DIR AND GPIOD_LIBRARIES)
        set(GPIO_AVAILABLE ON CACHE BOOL "Whether GPIO/libgpiod is available" FORCE)
        message(STATUS "libgpiod found, GPIO support enabled")
    else()
        message(WARNING "libgpiod not found, GPIO support disabled (simulation mode will use debugfs)")
    endif()
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

find_package(Threads REQUIRED)

target_link_libraries(drone_simulations PRIVATE nlohmann_json::nlohmann_json Threads::Threads)
if(ENABLE_HTTP)
target_link_libraries(drone_simulations PRIVATE httplib::httplib)
endif()
if(GPIO_AVAILABLE)
target_link_libraries(drone_simulations PRIVATE ${GPIOD_LIBRARIES})
target_include_directories(drone_simulations PRIVATE ${GPIOD_INCLUDE_DIRS})
endif()

target_include_directories(drone_simulations PRIVATE include)

if(ENABLE_HTTP)
target_compile_definitions(drone_simulations PRIVATE ENABLE_HTTP=1)
else()
target_compile_definitions(drone_simulations PRIVATE ENABLE_HTTP=0)
endif()

# HW22: Set USE_GPIOD when libgpiod is available
if(GPIO_AVAILABLE)
target_compile_definitions(drone_simulations PRIVATE USE_GPIOD=1)
else()
target_compile_definitions(drone_simulations PRIVATE USE_GPIOD=0)
endif()

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(drone_simulations PRIVATE -Wall -Wextra -pedantic)

# Опція для ThreadSanitizer: cmake --preset debug -DENABLE_TSAN=ON
# Для виявлення гонок даних між потоками.
option(ENABLE_TSAN "Build with ThreadSanitizer to detect data races" OFF)
if(ENABLE_TSAN)
    target_compile_options(drone_simulations PRIVATE -fsanitize=thread -g)
    target_link_options(drone_simulations PRIVATE -fsanitize=thread)
endif()
