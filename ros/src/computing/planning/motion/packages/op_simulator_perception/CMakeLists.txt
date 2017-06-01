cmake_minimum_required(VERSION 2.8.3)
project(op_simulator_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  runtime_manager
  lidar_tracker
  pcl_conversions
  pcl_ros
  map_file
  op_utility
  op_planner
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs runtime_manager map_file op_utility op_planner
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
runtime_manager_generate_messages_cpp
map_file_generate_messages_cpp
)
