cmake_minimum_required(VERSION 2.8.3)
project(ff_waypoint_follower)

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
add_executable(ff_waypoint_follower nodes/ff_waypoint_follower.cpp nodes/ff_waypoint_follower_core.cpp)
target_link_libraries(ff_waypoint_follower utilityh plannerh simuh ${catkin_LIBRARIES})

#link_directories(/usr/local/lib/zmp/minivan)
#add_executable(ff_waypoint_follower nodes/ff_waypoint_follower.cpp nodes/ff_waypoint_follower_core.cpp nodes/HevCnt.cpp nodes/HevComm.cpp)
#target_link_libraries(ff_waypoint_follower utilityh plannerh simuh CANUSB hevcontrol ${catkin_LIBRARIES})

add_dependencies(ff_waypoint_follower 
runtime_manager_generate_messages_cpp
cv_tracker_generate_messages_cpp
waypoint_follower_generate_messages_cpp
map_file_generate_messages_cpp
)
