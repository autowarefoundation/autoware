cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  tf
  tf_conversions
  pcl_ros
  sensor_msgs
  geometry_msgs
  autoware_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  tf
  tf_conversions
  pcl_ros
  sensor_msgs
  geometry_msgs
  autoware_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(astar_planner
  src/astar_planner.cpp
  src/astar_util.cpp
)

target_link_libraries(astar_planner
  ${catkin_LIBRARIES}
)

add_dependencies(astar_planner
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

install(TARGETS astar_planner
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
