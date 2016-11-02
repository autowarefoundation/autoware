cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf
  gnss
  runtime_manager
  waypoint_follower
  vector_map
  vehicle_socket
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libvelocity_set
  CATKIN_DEPENDS roscpp std_msgs tf runtime_manager waypoint_follower vehicle_socket vector_map
  DEPENDS gnss
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




add_library(libvelocity_set lib/libvelocity_set.cpp)
target_link_libraries(libvelocity_set ${catkin_LIBRARIES} vector_map)

add_executable(velocity_set nodes/velocity_set/velocity_set.cpp)
target_link_libraries(velocity_set libwaypoint_follower libvelocity_set ${catkin_LIBRARIES})
add_dependencies(velocity_set 
waypoint_follower_generate_messages_cpp
runtime_manager_generate_messages_cpp)

add_executable(obstacle_avoid nodes/obstacle_avoid/obstacle_avoid.cpp)
target_link_libraries(obstacle_avoid libwaypoint_follower  ${catkin_LIBRARIES})
add_dependencies(obstacle_avoid 
waypoint_follower_generate_messages_cpp)

