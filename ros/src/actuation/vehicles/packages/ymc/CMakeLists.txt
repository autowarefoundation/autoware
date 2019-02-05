cmake_minimum_required(VERSION 2.8.3)
project(ymc)

set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rosconsole
  autoware_msgs
  ds4_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  rosconsole
  autoware_msgs
  ds4_msgs
)

include_directories(
  ${autoware_msgs_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(g30esli_interface
  node/g30esli_interface/g30esli_interface.cpp
)

add_dependencies(g30esli_interface
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(g30esli_interface
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libg30esli.a
)
