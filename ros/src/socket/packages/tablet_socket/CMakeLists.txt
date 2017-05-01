cmake_minimum_required(VERSION 2.8.3)
project(tablet_socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vehicle_socket
  ndt_localizer
  message_generation
  tf
  gnss
  tablet_socket_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS vehicle_socket ndt_localizer tf
  DEPENDS gnss
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tablet_receiver nodes/tablet_receiver/tablet_receiver.cpp)
target_link_libraries(tablet_receiver ${catkin_LIBRARIES})
add_dependencies(tablet_receiver tablet_socket_generate_messages_cpp)

add_executable(tablet_sender nodes/tablet_sender/tablet_sender.cpp)
target_link_libraries(tablet_sender ${catkin_LIBRARIES})
add_dependencies(tablet_sender
  tablet_socket_generate_messages_cpp
  vehicle_socket_generate_messages_cpp
  ndt_localizer_generate_messages_cpp
)
