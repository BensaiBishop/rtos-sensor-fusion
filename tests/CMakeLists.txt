# tests/CMakeLists.txt

# Add include dirs for all tests
set(TEST_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/intern/include
    ${CMAKE_SOURCE_DIR}/intern/include/state
    ${CMAKE_SOURCE_DIR}/intern/include/tasks
    ${CMAKE_SOURCE_DIR}/intern/include/sim
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/include
    ${CMAKE_SOURCE_DIR}/extern/FreeRTOS-Kernel/portable/ThirdParty/GCC/Posix
)

# In tests/CMakeLists.txt, before add_executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)

# Dummy test
add_executable(dummy_test dummy_test.cpp)
target_include_directories(dummy_test PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(dummy_test PRIVATE gtest gtest_main pthread)
add_test(NAME dummy_test COMMAND dummy_test)

# Control test
add_executable(test_control test_control.cpp)
target_include_directories(test_control PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_control PRIVATE gtest gtest_main pthread)
add_test(NAME test_control COMMAND test_control)

# Sensors test
add_executable(test_sensors test_sensors.cpp
    ${CMAKE_SOURCE_DIR}/src/sim/trajectory.cpp
)
target_include_directories(test_sensors PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_sensors PRIVATE gtest gtest_main pthread)
add_test(NAME test_sensors COMMAND test_sensors)
