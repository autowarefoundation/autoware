cmake_minimum_required(VERSION 2.8.3)
project(ui_socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  Waypoint.msg
  gear_info.msg
  mode_info.msg
  route_info.msg
  error_info.msg
  can_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(ui_receiver nodes/ui_receiver/ui_receiver.cpp)
target_link_libraries(ui_receiver ${catkin_LIBRARIES})
add_dependencies(ui_receiver ui_socket_generate_messages_cpp)

add_executable(ui_sender nodes/ui_sender/ui_sender.cpp)
target_link_libraries(ui_sender ${catkin_LIBRARIES})
add_dependencies(ui_sender ui_socket_generate_messages_cpp)
