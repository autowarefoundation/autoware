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
  message_generation
  runtime_manager
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  lane.msg
  waypoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lf_func
  CATKIN_DEPENDS roscpp std_msgs tf runtime_manager
  DEPENDS gnss
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lf_func lib/lf_func.cpp)
add_dependencies(lf_func
lane_follower_generate_messages_cpp)

add_executable(pure_pursuit nodes/pure_pursuit/pure_pursuit.cpp)
target_link_libraries(pure_pursuit gnss lf_func ${catkin_LIBRARIES})
add_dependencies(pure_pursuit 
waypoint_follower_generate_messages_cpp
runtime_manager_generate_messages_cpp)

add_executable(odom_gen nodes/odom_gen/odom_gen.cpp)
target_link_libraries(odom_gen ${catkin_LIBRARIES})

add_executable(distance_keep nodes/distance_keep/distance_keep.cpp)
target_link_libraries(distance_keep lf_func ${catkin_LIBRARIES})
add_dependencies(distance_keep 
waypoint_follower_generate_messages_cpp)

add_executable(twist_through nodes/distance_keep/twist_through.cpp)
target_link_libraries(twist_through ${catkin_LIBRARIES})