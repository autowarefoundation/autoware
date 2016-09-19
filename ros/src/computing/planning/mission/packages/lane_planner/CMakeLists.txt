cmake_minimum_required(VERSION 2.8.3)
project(lane_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tablet_socket
  waypoint_follower
  runtime_manager
  vector_map
  gnss
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vmap
  CATKIN_DEPENDS tablet_socket waypoint_follower runtime_manager vector_map
  DEPENDS gnss
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vmap lib/lane_planner/vmap.cpp)
target_link_libraries(vmap vector_map ${catkin_LIBRARIES})
add_dependencies(vmap
  waypoint_follower_generate_messages_cpp
  tablet_socket_generate_messages_cpp
)

add_executable(lane_navi nodes/lane_navi/lane_navi.cpp)
target_link_libraries(lane_navi vmap ${catkin_LIBRARIES})
add_dependencies(lane_navi
  waypoint_follower_generate_messages_cpp
  tablet_socket_generate_messages_cpp
)

add_executable(lane_rule nodes/lane_rule/lane_rule.cpp)
target_link_libraries(lane_rule vmap ${catkin_LIBRARIES})
add_dependencies(lane_rule
  waypoint_follower_generate_messages_cpp
  runtime_manager_generate_messages_cpp
)

add_executable(lane_stop nodes/lane_stop/lane_stop.cpp)
target_link_libraries(lane_stop vmap ${catkin_LIBRARIES})
add_dependencies(lane_stop
  waypoint_follower_generate_messages_cpp
  runtime_manager_generate_messages_cpp
)

add_executable(lane_select nodes/lane_select/lane_select.cpp)
target_link_libraries(lane_select ${catkin_LIBRARIES})
add_dependencies(lane_select
  waypoint_follower_generate_messages_cpp
  runtime_manager_generate_messages_cpp
)
