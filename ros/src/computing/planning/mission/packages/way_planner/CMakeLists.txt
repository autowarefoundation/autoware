cmake_minimum_required(VERSION 2.8.3)
project(way_planner)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  geometry_msgs
  vector_map_msgs
  roscpp
  rospy
  std_msgs
  waypoint_follower
  message_generation
  tf
  jsk_recognition_msgs
  pcl_conversions
  pcl_ros
  op_utility
  op_planner
  op_simu
  autoware_msgs
)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES way_planner
  CATKIN_DEPENDS geometry_msgs vector_map_msgs roscpp rospy std_msgs waypoint_follower op_utility op_planner op_simu
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include include/plannerh
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ executable
 add_executable(way_planner nodes/way_planner.cpp nodes/way_planner_core.cpp nodes/RosHelpers.cpp nodes/SocketServer.cpp)
 # add_executable(way_planner nodes/kml_map_recorder.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(way_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(way_planner ${catkin_LIBRARIES} )
