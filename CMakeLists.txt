cmake_minimum_required(VERSION 3.16)
project(rtos_sensor_fusion LANGUAGES C CXX)

# C++ standard settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable warnings for GCC/Clang
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
set(INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/intern/include
    ${CMAKE_SOURCE_DIR}/intern/include/state
    ${CMAKE_SOURCE_DIR}/intern/include/tasks
    ${CMAKE_SOURCE_DIR}/intern/include/sim
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/include
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/portable/ThirdParty/GCC/Posix
)

# Source files
set(SRC_FILES
    # Project sources
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/rtos_demo.cpp
    ${CMAKE_SOURCE_DIR}/src/state/state.cpp
    ${CMAKE_SOURCE_DIR}/src/tasks/sensor_task.cpp
    ${CMAKE_SOURCE_DIR}/src/tasks/estimator_task.cpp
    ${CMAKE_SOURCE_DIR}/src/tasks/control_task.cpp
    ${CMAKE_SOURCE_DIR}/src/tasks/watchdog_task.cpp
    ${CMAKE_SOURCE_DIR}/src/tasks/dashboard_task.cpp
    ${CMAKE_SOURCE_DIR}/src/sim/trajectory.cpp

    # FreeRTOS kernel core
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/tasks.c
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/list.c
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/queue.c
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/timers.c
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/event_groups.c

    # FreeRTOS dynamic memory
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/portable/MemMang/heap_4.c

    # POSIX/Linux port
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/portable/ThirdParty/GCC/Posix/port.c
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/portable/ThirdParty/GCC/Posix/utils/wait_for_event.c
)

# Executable target
add_executable(rtos-sensor-fusion ${SRC_FILES})
target_include_directories(rtos-sensor-fusion PRIVATE ${INCLUDE_DIRS})
target_link_libraries(rtos-sensor-fusion PRIVATE pthread)

# Download and build GoogleTest (requires CMake 3.11+)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enable testing and add tests subdirectory
enable_testing()
add_subdirectory(tests)
