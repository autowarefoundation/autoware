cmake_minimum_required(VERSION 2.8.3)
project(ymc)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

find_library(ymc_can NAMES libymc_can.a PATHS lib)

add_executable(g30esli_interface
  node/g30esli_interface/g30esli_interface.cpp
  )

target_link_libraries(g30esli_interface
  ${catkin_LIBRARIES}
  ${ymc_can}
  )
