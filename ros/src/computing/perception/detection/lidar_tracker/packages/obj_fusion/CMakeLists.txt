cmake_minimum_required(VERSION 2.8.3)
project(obj_fusion)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(vector_map REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  jsk_recognition_msgs
  vector_map_server
  autoware_msgs
  vector_map
  )


set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")
catkin_package(
  CATKIN_DEPENDS 
  roscpp
  autoware_msgs
  tf
  jsk_recognition_msgs
  vector_map
  vector_map_server
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${vector_map_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
)

#Object Fusion
add_executable(obj_fusion 
  nodes/obj_fusion/obj_fusion.cpp
  )

target_link_libraries(obj_fusion
  ${catkin_LIBRARIES}
  m
  )
add_dependencies(obj_fusion
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS obj_fusion
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
