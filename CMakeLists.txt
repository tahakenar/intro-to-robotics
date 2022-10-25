cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

project(robotics)

include_directories(include)
include_directories(eigen)

enable_testing()

add_subdirectory(googletest)

set(HEADERS include/transforms.h include/forward_kinematics.h)
set(SOURCES src/transforms.cpp src/forward_kinematics.cpp)

add_library(robotics_lib STATIC ${SOURCES} ${HEADERS})

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} robotics_lib)

add_subdirectory(test)
