cmake_minimum_required(VERSION 2.8.3)
project(ff_waypoint_follower)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  runtime_manager
  map_file
  op_utility
  op_planner
  op_simu
  waypoint_follower
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
  CATKIN_DEPENDS roscpp geometry_msgs runtime_manager map_file op_utility op_planner op_simu waypoint_follower autoware_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include 
#  /usr/local/include/zmp/minivan
  ${catkin_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(libplanner_x lib/libplanner_x.cpp)

link_directories(lib)
add_executable(ff_waypoint_follower nodes/ff_waypoint_follower.cpp nodes/ff_waypoint_follower_core.cpp)
target_link_libraries(ff_waypoint_follower
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )

#link_directories(/usr/local/lib/zmp/minivan)
#add_executable(ff_waypoint_follower nodes/ff_waypoint_follower.cpp nodes/ff_waypoint_follower_core.cpp nodes/HevCnt.cpp nodes/HevComm.cpp)
#target_link_libraries(ff_waypoint_follower utilityh plannerh simuh CANUSB hevcontrol ${catkin_LIBRARIES})

add_dependencies(ff_waypoint_follower 
${catkin_EXPORTED_TARGETS}
)
