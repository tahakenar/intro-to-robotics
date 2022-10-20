cmake_minimum_required(VERSION 3.23.1)
set (CMAKE_CXX_STANDARD 14)

project(robotics)

include_directories(include)

find_package(Eigen3 REQUIRED NO_MODULE)
link_directories(${Eigen_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp src/transforms.cpp src/forward_kinematics.cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
