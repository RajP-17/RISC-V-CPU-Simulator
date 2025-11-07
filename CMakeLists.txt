cmake_minimum_required(VERSION 3.22)
project(rv32_pipeline_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Core simulation library
add_library(simcore
    sim/src/isa.cpp
    sim/src/memory.cpp
    sim/src/cpu.cpp
    sim/src/loader.cpp
    sim/src/checks.cpp
)

target_include_directories(simcore PUBLIC sim/include)
target_compile_options(simcore PRIVATE -Wall -Wextra -Wpedantic)

# Part 1: Single CPU executable
add_executable(single sim/src/main_single.cpp)
target_link_libraries(single PRIVATE simcore)
target_compile_options(single PRIVATE -Wall -Wextra -Wpedantic)

# Set output directory
set_target_properties(single PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

# Platform-specific settings
if(MSVC)
    target_compile_options(simcore PRIVATE /W4)
    target_compile_options(single PRIVATE /W4)
else()
    target_compile_options(simcore PRIVATE -O2)
    target_compile_options(single PRIVATE -O2)
endif()
