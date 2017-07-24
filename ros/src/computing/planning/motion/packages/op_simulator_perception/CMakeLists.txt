cmake_minimum_required(VERSION 2.8.3)
project(op_simulator_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  pcl_conversions
  pcl_ros
  op_utility
  op_planner
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
  CATKIN_DEPENDS roscpp geometry_msgs op_utility op_planner autoware_msgs
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
add_executable(op_simulator_perception nodes/OpenPlannerSimulatorPerception.cpp nodes/OpenPlannerSimulatorPerception_core.cpp)
target_link_libraries(op_simulator_perception ${catkin_LIBRARIES})

add_dependencies(op_simulator_perception 
autoware_msgs_generate_messages_cpp
)
