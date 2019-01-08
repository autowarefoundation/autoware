cmake_minimum_required(VERSION 2.8.12)
project(obj_reproj)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  sensor_msgs
  std_msgs
  autoware_msgs
  jsk_recognition_msgs
  tf
  )

FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  sensor_msgs
  std_msgs
  autoware_msgs
  jsk_recognition_msgs
  tf
  )

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  lib
  include
  )

## obj_reproj ##
add_executable(obj_reproj
  nodes/obj_reproj/obj_reproj.cpp
  )

target_link_libraries(obj_reproj
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )
add_dependencies(obj_reproj
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS obj_reproj
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
