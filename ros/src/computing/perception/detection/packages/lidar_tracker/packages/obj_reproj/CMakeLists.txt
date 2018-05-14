cmake_minimum_required(VERSION 2.8.12)
project(obj_reproj)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  autoware_msgs
  tf
  gnss
  )

FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  sensor_msgs
  std_msgs
  autoware_msgs
  tf
  gnss
  )

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  lib
  include
  )

## obj_reproj ##
add_executable(obj_reproj
  nodes/obj_reproj/obj_reproj.cpp
  )

set_target_properties(obj_reproj
  PROPERTIES COMPILE_FLAGS
  "-DHAVE_JSK_PLUGIN"
  )

target_link_libraries(obj_reproj
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

install(TARGETS obj_reproj
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
