cmake_minimum_required(VERSION 2.8.3)
project(tablet_socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  vehicle_socket
  message_generation
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  Waypoint.msg
  gear_cmd.msg
  mode_cmd.msg
  route_cmd.msg
  error_info.msg
  mode_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS vehicle_socket
)

add_executable(tablet_receiver nodes/tablet_receiver/tablet_receiver.cpp)
target_link_libraries(tablet_receiver ${catkin_LIBRARIES})
add_dependencies(tablet_receiver tablet_socket_generate_messages_cpp)

add_executable(tablet_sender nodes/tablet_sender/tablet_sender.cpp)
target_link_libraries(tablet_sender ${catkin_LIBRARIES})
add_dependencies(tablet_sender
  tablet_socket_generate_messages_cpp
  vehicle_socket_generate_messages_cpp
)
