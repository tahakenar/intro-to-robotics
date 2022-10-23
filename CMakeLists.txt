cmake_minimum_required(VERSION 3.23.1)
set(CMAKE_CXX_STANDARD 14)

project(robotics)

include_directories(include)

find_package(Eigen3 REQUIRED NO_MODULE)
link_directories(${Eigen_INCLUDE_DIRS})

enable_testing()

add_subdirectory(googletest)

set(HEADERS include/transforms.h include/forward_kinematics.h)
set(SOURCES src/transforms.cpp src/forward_kinematics.cpp)

add_library(robotics_lib STATIC ${SOURCES} ${HEADERS})
target_link_libraries(robotics_lib Eigen3::Eigen)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} robotics_lib)

add_subdirectory(test)
