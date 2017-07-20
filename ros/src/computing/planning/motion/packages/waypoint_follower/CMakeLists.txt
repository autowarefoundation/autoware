cmake_minimum_required(VERSION 2.8.3)
project(waypoint_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf
  gnss
  autoware_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libwaypoint_follower
  CATKIN_DEPENDS roscpp tf autoware_msgs
 # DEPENDS
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(libwaypoint_follower lib/libwaypoint_follower.cpp)
add_dependencies(libwaypoint_follower
${catkin_EXPORTED_TARGETS}
)

add_executable(pure_pursuit
  nodes/pure_pursuit/pure_pursuit_node.cpp
  nodes/pure_pursuit/pure_pursuit.cpp
  nodes/pure_pursuit/pure_pursuit_core.cpp
  nodes/pure_pursuit/pure_pursuit_viz.cpp
  )
target_link_libraries(pure_pursuit libwaypoint_follower ${catkin_LIBRARIES})
add_dependencies(pure_pursuit
${catkin_EXPORTED_TARGETS} )

add_executable(wf_simulator nodes/wf_simulator/wf_simulator.cpp)
target_link_libraries(wf_simulator libwaypoint_follower ${catkin_LIBRARIES})

add_executable(twist_filter nodes/twist_filter/twist_filter.cpp)
target_link_libraries(twist_filter ${catkin_LIBRARIES})
add_dependencies(twist_filter
${catkin_EXPORTED_TARGETS} )


