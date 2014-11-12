cmake_minimum_required(VERSION 2.8.3)
project(input)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  gearData.msg
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

add_executable(gear input/gear.cpp)
target_link_libraries(gear ${catkin_LIBRARIES})
add_dependencies(gear input_generate_messages_cpp)
