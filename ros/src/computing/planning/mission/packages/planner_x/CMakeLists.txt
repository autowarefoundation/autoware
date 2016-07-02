cmake_minimum_required(VERSION 2.8.3)
project(planner_x)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  runtime_manager
  cv_tracker
  map_file
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs runtime_manager cv_tracker map_file
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include include/planner_x
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(libplanner_x lib/libplanner_x.cpp)

link_directories(lib)

add_executable(planner_x nodes/planner_x.cpp nodes/planner_x_core.cpp nodes/RosHelpers.cpp)
target_link_libraries(planner_x utilityh plannerh ${catkin_LIBRARIES})
add_dependencies(planner_x 
runtime_manager_generate_messages_cpp
cv_tracker_generate_messages_cpp
waypoint_follower_generate_messages_cpp
map_file_generate_messages_cpp
)
