cmake_minimum_required(VERSION 2.8.3)
project(lane_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  visualization_msgs
  ui_socket
  lane_follower
  gnss
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS ui_socket lane_follower
  DEPENDS gnss
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vmap_parser lib/vmap_parser.cpp)

add_executable(lane_navi nodes/lane_navi/lane_navi.cpp)
target_link_libraries(lane_navi vmap_parser gnss ${catkin_LIBRARIES})
add_dependencies(lane_navi ui_socket_generate_messages_cpp)

add_executable(lane_rule nodes/lane_rule/lane_rule.cpp)
target_link_libraries(lane_rule ${catkin_LIBRARIES})
add_dependencies(lane_rule lane_follower_generate_messages_cpp)
