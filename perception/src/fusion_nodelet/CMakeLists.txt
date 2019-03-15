cmake_minimum_required(VERSION 2.8.3)
project(fusion_nodelet)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11)
# add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
# set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "DEBUG")

find_package(catkin REQUIRED COMPONENTS
  # nodelet
  roscpp
  std_msgs
  tf
  ndt_omp
  message_filters pcl_ros pcl_conversions pcl_msgs sensor_msgs
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hdl_scan_matching_odometry
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories("/usr/include/eigen3")
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# nodelets-fusion_cloud_nodelet
add_library(fusion_cloud_nodelet 
  src/fusion_cloud_nodelet.cc 
)
target_link_libraries(fusion_cloud_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# nodelets-perception_nodelet
add_library(perception_nodelet 
  src/perception_nodelet.cc 
  src/clustering.h
  src/clustering.cc
  src/filter.h
  src/filter.cc
)
target_link_libraries(perception_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


