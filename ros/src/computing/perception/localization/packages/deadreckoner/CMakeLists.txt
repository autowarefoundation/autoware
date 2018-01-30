cmake_minimum_required(VERSION 2.8.3)
project(deadreckoner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES deadreckoner
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(deadreckoner
  nodes/deadreckoner_node.cpp
  nodes/deadreckoner.cpp
)

target_link_libraries(deadreckoner
  ${catkin_LIBRARIES}
)
