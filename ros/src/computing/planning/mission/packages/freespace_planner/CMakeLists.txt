cmake_minimum_required(VERSION 2.8.3)
project(freespace_planner)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  nav_msgs
  visualization_msgs
  autoware_msgs
  tf
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS autoware_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_executable(astar_navi nodes/astar_navi/astar_navi.cpp nodes/astar_navi/astar_search.cpp nodes/astar_navi/search_info_ros.cpp include/astar_util.cpp)
target_link_libraries(astar_navi ${catkin_LIBRARIES})
add_dependencies(astar_navi ${catkin_EXPORTED_TARGETS})
