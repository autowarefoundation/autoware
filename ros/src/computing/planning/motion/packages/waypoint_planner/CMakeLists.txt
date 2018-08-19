cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planner)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  tf
  waypoint_follower
  autoware_msgs
  vector_map
  astar_planner
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  tf
  waypoint_follower
  autoware_msgs
  vector_map
  astar_planner
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_set
  nodes/velocity_set/velocity_set.cpp
  nodes/velocity_set/velocity_set_path.cpp
  nodes/velocity_set/velocity_set_info.cpp
  nodes/velocity_set/libvelocity_set.cpp
)

target_link_libraries(velocity_set
  ${catkin_LIBRARIES}
)

add_dependencies(velocity_set
  ${catkin_EXPORTED_TARGETS}
)

add_executable(obstacle_avoid
  nodes/obstacle_avoid/obstacle_avoid.cpp
  nodes/obstacle_avoid/search_info_ros.cpp
)

target_link_libraries(obstacle_avoid
  ${catkin_LIBRARIES}
)

add_dependencies(obstacle_avoid
  ${catkin_EXPORTED_TARGETS}
)
