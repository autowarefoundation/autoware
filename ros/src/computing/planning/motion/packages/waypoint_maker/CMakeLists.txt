cmake_minimum_required(VERSION 2.8.3)
project(waypoint_maker)

find_package(catkin REQUIRED COMPONENTS
  gnss
  roscpp
  std_msgs
  tf
  vehicle_socket
  waypoint_follower
  runtime_manager
  map_file
  lane_planner
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_follower
  CATKIN_DEPENDS roscpp std_msgs tf vehicle_socket waypoint_follower runtime_manager map_file lane_planner
  DEPENDS gnss waypoint_follower
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_loader nodes/waypoint_loader/waypoint_loader.cpp)
target_link_libraries(waypoint_loader libwaypoint_follower ${catkin_LIBRARIES})
add_dependencies(waypoint_loader
  waypoint_follower_generate_messages_cpp
  runtime_manager_generate_messages_cpp)

add_executable(waypoint_saver nodes/waypoint_saver/waypoint_saver.cpp)
target_link_libraries(waypoint_saver libwaypoint_follower ${catkin_LIBRARIES})
add_dependencies(waypoint_saver
  waypoint_follower_generate_messages_cpp
  vehicle_socket_generate_messages_cpp)

add_executable(waypoint_clicker nodes/waypoint_clicker/waypoint_clicker.cpp)
target_link_libraries(waypoint_clicker vmap ${catkin_LIBRARIES})
add_dependencies(waypoint_clicker
  map_file_generate_messages_cpp)

add_executable(waypoint_marker_publisher nodes/waypoint_marker_publisher/waypoint_marker_publisher.cpp)
target_link_libraries(waypoint_marker_publisher libwaypoint_follower ${catkin_LIBRARIES})
add_dependencies(waypoint_marker_publisher
  waypoint_follower_generate_messages_cpp
  runtime_manager_generate_messages_cpp)
