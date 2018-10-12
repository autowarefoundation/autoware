cmake_minimum_required(VERSION 2.8.3)
project(freespace_planner)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  tf
  nav_msgs
  geometry_msgs
  visualization_msgs
  autoware_msgs
  astar_search
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  visualization_msgs
  autoware_msgs
  astar_search
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_navi
  nodes/astar_navi/astar_navi.cpp
  nodes/astar_navi/astar_navi_node.cpp
)

target_link_libraries(astar_navi
  ${catkin_LIBRARIES}
)

add_dependencies(astar_navi
  ${catkin_EXPORTED_TARGETS}
)
