cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

set(TF_TEST transformation_tests)

set(SOURCES transformation_tests.cpp)

add_executable(${TF_TEST} ${SOURCES})
target_link_libraries(${TF_TEST} PUBLIC gtest_main robotics_lib)

add_test(NAME ${TF_TEST} COMMAND ${TF_TEST})

set(FK_TEST forward_kine_tests)

set(SOURCES forward_kine_tests.cpp)

add_executable(${FK_TEST} ${SOURCES})
target_link_libraries(${FK_TEST} PUBLIC gtest_main robotics_lib)

add_test(NAME ${FK_TEST} COMMAND ${FK_TEST})
