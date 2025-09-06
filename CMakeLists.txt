cmake_minimum_required(VERSION 3.16)
project(rtos_sensor_fusion LANGUAGES CXX)

# Use modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable warnings 
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Source files 
add_executable(rtos-sensor-fusion
    src/main.cpp
)

# GoogleTest setup 
enable_testing()
add_subdirectory(tests)