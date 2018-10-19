cmake_minimum_required(VERSION 2.8.3)
project(as)

# If submodule is not cloned this project will not be build.
set(AS_MSG_PATH "${CMAKE_SOURCE_DIR}/msgs/platform_automation_msgs/module_comm_msgs")
if(EXISTS "${AS_MSG_PATH}")

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  std_msgs
  geometry_msgs
  module_comm_msgs
  platform_comm_msgs
  dbw_mkz_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  module_comm_msgs
  platform_comm_msgs
  dbw_mkz_msgs
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(as_interface
  nodes/as_interface/as_interface_node.cpp
  nodes/as_interface/as_interface.cpp
  )
target_link_libraries(as_interface ${catkin_LIBRARIES})
add_dependencies(as_interface ${catkin_EXPORTED_TARGETS} )

endif()
