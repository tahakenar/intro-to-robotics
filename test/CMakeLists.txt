cmake_minimum_required(VERSION 3.23.1)
set(CMAKE_CXX_STANDARD 14)

set(THIS robotics_tests)

set(SOURCES robotics_tests.cpp)

add_executable(${THIS} ${SOURCES})
target_link_libraries(${THIS} PUBLIC gtest_main robotics_lib)

add_test(NAME ${THIS} COMMAND ${THIS})
