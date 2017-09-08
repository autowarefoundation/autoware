cmake_minimum_required(VERSION 2.8.12)
project(camera_lidar_calibration)

include(FindPkgConfig)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  autoware_msgs
)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL)

catkin_package(CATKIN_DEPENDS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
        autoware_msgs
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#camera lidar node
add_executable(camera_lidar_calibration_node
        src/camera_lidar_calibration_node.cpp
)
target_link_libraries(camera_lidar_calibration_node
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)