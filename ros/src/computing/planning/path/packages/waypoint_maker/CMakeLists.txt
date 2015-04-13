cmake_minimum_required(VERSION 2.8.3)
project(waypoint_maker)

find_package(catkin REQUIRED COMPONENTS
  gnss
  roscpp
  std_msgs
  tf
  vehicle_socket
  lane_follower
  lane_planner
  runtime_manager
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_follower
  CATKIN_DEPENDS roscpp std_msgs tf vehicle_socket lane_follower lane_planner runtime_manager
  DEPENDS gnss
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_loader nodes/waypoint_loader/waypoint_loader.cpp)
target_link_libraries(waypoint_loader vmap_parser ${catkin_LIBRARIES})
add_dependencies(waypoint_loader
  lane_follower_generate_messages_cpp
  runtime_manager_generate_messages_cpp)

add_executable(waypoint_saver nodes/waypoint_saver/waypoint_saver.cpp)
target_link_libraries(waypoint_saver gnss ${catkin_LIBRARIES})
add_dependencies(waypoint_saver
  lane_follower_generate_messages_cpp
  vehicle_socket_generate_messages_cpp)
