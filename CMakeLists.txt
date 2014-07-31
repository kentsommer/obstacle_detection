cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#Find PCL 1.7
find_package(PCL 1.7 REQUIRED)
include_directories(BEFORE ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
if (NOT PCL_FOUND)
    MESSAGE(FATAL_ERROR "PCL not found.\n")
endif (NOT PCL_FOUND)

message(STATUS "pcl libraries... ${PCL_LIBRARIES}")

add_executable(obstacle_detection src/obstacle_detection.cpp)
target_link_libraries(obstacle_detection ${PCL_LIBRARIES})
target_link_libraries(obstacle_detection ${catkin_LIBRARIES})
target_link_libraries(obstacle_detection ${pcl_segmentation})
