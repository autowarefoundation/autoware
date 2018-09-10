cmake_minimum_required(VERSION 2.8.3)
project(lidar_apollo_cnn_seg_detect)

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

find_package(Caffe)

catkin_package(
  INCLUDE_DIRS
                include
  CATKIN_DEPENDS
                roscpp
                tf
                tf_conversions
                pcl_ros
                sensor_msgs
                geometry_msgs
                autoware_msgs
  DEPENDS
                Caffe
)

include_directories(
  include
  nodes
  ${catkin_INCLUDE_DIRS}
  ${CAFFE_INCLUDEDIR}
)

add_executable(lidar_apollo_cnn_seg_detect
  nodes/lidar_apollo_cnn_seg_detect_node.cpp
  nodes/lidar_apollo_cnn_seg_detect.cpp
)
target_link_libraries(lidar_apollo_cnn_seg_detect
  ${catkin_LIBRARIES}
  ${Caffe_LIBRARIES}
)
add_dependencies(lidar_apollo_cnn_seg_detect
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
  lidar_apollo_cnn_seg_detect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
