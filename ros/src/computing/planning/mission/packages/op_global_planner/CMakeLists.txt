cmake_minimum_required(VERSION 2.8.3)
project(op_global_planner)

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
  op_ros_helpers
  autoware_msgs
)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op_global_planner
  CATKIN_DEPENDS geometry_msgs vector_map_msgs roscpp rospy std_msgs waypoint_follower op_utility op_planner op_simu op_ros_helpers
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ executable
 add_executable(op_global_planner nodes/op_global_planner.cpp nodes/op_global_planner_core.cpp)
target_link_libraries(op_global_planner ${catkin_LIBRARIES} )

add_dependencies(op_global_planner ${catkin_EXPORTED_TARGETS})
