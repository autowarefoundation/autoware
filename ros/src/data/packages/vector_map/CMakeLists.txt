cmake_minimum_required(VERSION 2.8.3)
project(vector_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  vector_map_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vector_map
  CATKIN_DEPENDS roscpp tf geometry_msgs vector_map_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vector_map lib/vector_map/vector_map.cpp)
add_dependencies(vector_map
  geometry_msgs_generate_messages_cpp
  vector_map_msgs_generate_messages_cpp
)

add_executable(sample1 nodes/sample1/sample1.cpp)
target_link_libraries(sample1 vector_map ${catkin_LIBRARIES})
add_dependencies(sample1 vector_map_msgs_generate_messages_cpp)

add_executable(sample2 nodes/sample2/sample2.cpp)
target_link_libraries(sample2 vector_map ${catkin_LIBRARIES})
add_dependencies(sample2 vector_map_msgs_generate_messages_cpp)

add_executable(sample3 nodes/sample3/sample3.cpp)
target_link_libraries(sample3 vector_map ${catkin_LIBRARIES})
add_dependencies(sample3 vector_map_msgs_generate_messages_cpp)
