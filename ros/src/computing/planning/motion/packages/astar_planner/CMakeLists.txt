cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  tf
  runtime_manager
  waypoint_follower
  waypoint_follower_msgs
  vector_map
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf runtime_manager waypoint_follower waypoint_follower_msgs vector_map
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_set nodes/velocity_set/velocity_set.cpp nodes/velocity_set/velocity_set_path.cpp nodes/velocity_set/velocity_set_info.cpp nodes/velocity_set/libvelocity_set.cpp)
target_link_libraries(velocity_set ${catkin_LIBRARIES})
add_dependencies(velocity_set 
waypoint_follower_msgs_generate_messages_cpp
runtime_manager_generate_messages_cpp)

add_executable(obstacle_avoid nodes/obstacle_avoid/obstacle_avoid.cpp)
target_link_libraries(obstacle_avoid  ${catkin_LIBRARIES})
add_dependencies(obstacle_avoid 
waypoint_follower_msgs_generate_messages_cpp)


