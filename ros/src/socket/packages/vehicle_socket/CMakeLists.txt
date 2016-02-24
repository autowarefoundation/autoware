cmake_minimum_required(VERSION 2.8.3)
project(vehicle_socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  runtime_manager
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  CanInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS runtime_manager
)

add_executable(vehicle_receiver nodes/vehicle_receiver/vehicle_receiver.cpp)
target_link_libraries(vehicle_receiver ${catkin_LIBRARIES})
add_dependencies(vehicle_receiver
  vehicle_socket_generate_messages_cpp
  tablet_socket_generate_messages_cpp
)

add_executable(vehicle_sender nodes/vehicle_sender/vehicle_sender.cpp)
target_link_libraries(vehicle_sender ${catkin_LIBRARIES})
add_dependencies(vehicle_sender
  vehicle_socket_generate_messages_cpp
  tablet_socket_generate_messages_cpp
  runtime_manager_generate_messages_cpp
)
