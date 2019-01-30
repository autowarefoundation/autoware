cmake_minimum_required(VERSION 2.8.3)
project(as)

find_package(
  catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  message_filters
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  pacmod_msgs
  autoware_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  message_filters
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  pacmod_msgs
  autoware_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ssc_interface
  nodes/ssc_interface/ssc_interface_node.cpp
  nodes/ssc_interface/ssc_interface.cpp
)

target_link_libraries(ssc_interface ${catkin_LIBRARIES})

add_dependencies(ssc_interface ${catkin_EXPORTED_TARGETS})

install(TARGETS ssc_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
