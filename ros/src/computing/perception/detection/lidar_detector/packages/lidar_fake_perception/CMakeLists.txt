cmake_minimum_required(VERSION 2.8.3)
project(lidar_fake_perception)

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

add_executable(lidar_fake_perception_node
  nodes/lidar_fake_perception.cpp
  nodes/lidar_fake_perception_node.cpp
)
target_link_libraries(lidar_fake_perception_node
  ${catkin_LIBRARIES}
)
add_dependencies(lidar_fake_perception_node
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
  lidar_fake_perception_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
