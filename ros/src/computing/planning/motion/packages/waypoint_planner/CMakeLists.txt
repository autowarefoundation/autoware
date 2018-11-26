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
  astar_search
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  tf
  waypoint_follower
  autoware_msgs
  vector_map
  astar_search
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_avoid
  src/astar_avoid/astar_avoid.cpp
  src/astar_avoid/astar_avoid_node.cpp
)

target_link_libraries(astar_avoid
  ${catkin_LIBRARIES}
)

add_dependencies(astar_avoid
  ${catkin_EXPORTED_TARGETS}
)

add_executable(velocity_set
  src/velocity_set/velocity_set.cpp
  src/velocity_set/velocity_set_path.cpp
  src/velocity_set/velocity_set_info.cpp
  src/velocity_set/libvelocity_set.cpp
)

target_link_libraries(velocity_set
  ${catkin_LIBRARIES}
)

add_dependencies(velocity_set
  ${catkin_EXPORTED_TARGETS}
)
