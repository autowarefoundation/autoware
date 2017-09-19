cmake_minimum_required(VERSION 2.8.3)
project(mqtt_socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_msgs
  runtime_manager
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs runtime_manager autoware_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mqtt_sender nodes/mqtt_sender/mqtt_sender.cpp)
target_link_libraries(mqtt_sender -lmosquitto ${catkin_LIBRARIES})
add_dependencies(mqtt_sender
  autoware_msgs_generate_messages_cpp
)
