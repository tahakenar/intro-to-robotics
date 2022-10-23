cmake_minimum_required(VERSION 3.23.1)
set(CMAKE_CXX_STANDARD 14)

set(THIS robotics_tests)

set(SOURCES robotics_tests.cpp)

find_package(Eigen3 REQUIRED NO_MODULE)
link_directories(${Eigen_INCLUDE_DIRS})

add_executable(${THIS} ${SOURCES})
target_link_libraries(${THIS} PUBLIC gtest_main robotics_lib Eigen3::Eigen)

add_test(NAME ${THIS} COMMAND ${THIS})
