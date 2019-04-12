cmake_minimum_required(VERSION 3.10)
project(calib_lidar_imu)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(calib_lidar_imu main.cpp)
target_link_libraries (calib_lidar_imu ${PCL_LIBRARIES})