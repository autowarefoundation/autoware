cmake_minimum_required(VERSION 2.8.3)
project(ui_receiver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  gear_info.msg
  mode_info.msg
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

add_executable(ui_receiver ui_receiver/ui_receiver.cpp)
target_link_libraries(ui_receiver ${catkin_LIBRARIES})
add_dependencies(ui_receiver ui_receiver_generate_messages_cpp)
