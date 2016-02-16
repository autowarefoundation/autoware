cmake_minimum_required(VERSION 2.8.3)
project(freespace_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  visualization_msgs
  tablet_socket
  waypoint_follower
  runtime_manager
  map_file
  gnss
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS tablet_socket waypoint_follower runtime_manager map_file
  DEPENDS gnss
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_navi nodes/astar_navi/astar_navi.cpp)
target_link_libraries(astar_navi ${catkin_LIBRARIES})
add_dependencies(astar_navi waypoint_follower_generate_messages_cpp)
