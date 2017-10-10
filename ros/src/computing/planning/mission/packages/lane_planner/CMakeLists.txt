cmake_minimum_required(VERSION 2.8.3)
project(lane_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tablet_socket
  waypoint_follower
  vector_map
  gnss
  autoware_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vmap
  CATKIN_DEPENDS tablet_socket waypoint_follower vector_map autoware_msgs
  DEPENDS gnss
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vmap lib/lane_planner/vmap.cpp)
target_link_libraries(vmap ${catkin_LIBRARIES})
add_dependencies(vmap
  autoware_msgs_generate_messages_cpp
)

add_executable(lane_navi nodes/lane_navi/lane_navi.cpp)
target_link_libraries(lane_navi vmap ${catkin_LIBRARIES})
add_dependencies(lane_navi
  autoware_msgs_generate_messages_cpp
)

add_executable(lane_rule nodes/lane_rule/lane_rule.cpp)
target_link_libraries(lane_rule vmap ${catkin_LIBRARIES})
add_dependencies(lane_rule
  autoware_msgs_generate_messages_cpp
)

add_executable(lane_stop nodes/lane_stop/lane_stop.cpp)
target_link_libraries(lane_stop vmap ${catkin_LIBRARIES})
add_dependencies(lane_stop
  autoware_msgs_generate_messages_cpp
)

add_executable(lane_select nodes/lane_select/lane_select_node.cpp nodes/lane_select/lane_select_core.cpp nodes/lane_select/hermite_curve.cpp)
target_link_libraries(lane_select ${catkin_LIBRARIES})
add_dependencies(lane_select
  autoware_msgs_generate_messages_cpp
)
