cmake_minimum_required(VERSION 2.8.3)
project(op_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  pcl_conversions
  pcl_ros
  op_utility
  op_planner
  op_simu
  autoware_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs cv_tracker op_utility op_planner op_simu autoware_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include 
#  /usr/local/include/zmp/minivan
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(libplanner_x lib/libplanner_x.cpp)

link_directories(lib)
add_executable(op_simulator nodes/OpenPlannerSimulator.cpp nodes/OpenPlannerSimulator_core.cpp nodes/PolygonGenerator.cpp)
target_link_libraries(op_simulator ${catkin_LIBRARIES})

