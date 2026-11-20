cmake_minimum_required(VERSION 3.22)

project(stm32l4s5_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

set(TEST_SOURCES
    test_split_float.cpp
    test_hts221.cpp
    test_post_check.cpp
    test_stall_tracker.cpp
)

# Host binary. Requires a host C++ toolchain (g++/clang++). The dev container
# image must include g++ (see .devcontainer.json "build.args" / apt packages).
# The on-target variant of these suites is compiled by the root firmware
# CMakeLists when STM32_SELF_TEST=ON.
add_executable(unit_tests
    host_main.cpp
    ${TEST_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/humidity_temp_snsr.cpp
)
target_include_directories(unit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
add_test(NAME unit_tests COMMAND unit_tests)
