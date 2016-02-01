cmake_minimum_required(VERSION 2.8.3)
project(points2image)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosinterface
  std_msgs
  sensor_msgs
  rosinterface
  fastvirtualscan
  message_generation
  cv_bridge
  tf
  pcl_ros
  pcl_conversions
  calibration_camera_lidar
)

find_package(OpenCV REQUIRED)

pkg_check_modules(Qt5Core REQUIRED Qt5Core)
pkg_check_modules(Qt5Widgets REQUIRED Qt5Widgets)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  PointsImage.msg
#  CameraExtrinsic.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_lidar3d
   CATKIN_DEPENDS rosinterface calibration_camera_lidar
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result ${CMAKE_CXX_FLAGS}")

EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# library
add_library(points_image
  lib/points_image/points_image.cpp
)
add_dependencies(points_image points2image_generate_messages_cpp)

# points2vscan
add_custom_command(
  OUTPUT ui_mainwindow.h
  COMMAND
  ${Qt5BIN}/uic
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/points2vscan/ui_mainwindow.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/points2vscan/mainwindow.ui"
)
add_custom_target(points2vscan_ui_mainwindow DEPENDS ui_mainwindow.h)

add_custom_command(
  OUTPUT mainwindow_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/points2vscan/mainwindow_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/points2vscan/mainwindow.h"
)
add_custom_target(points2vscan_mainwindow_moc DEPENDS mainwindow_moc.cpp)

## Declare a cpp executable
add_executable(points2vscan
  nodes/points2vscan/main.cpp
  nodes/points2vscan/mainwindow.cpp
)

set_target_properties(points2vscan
  PROPERTIES COMPILE_FLAGS "-fPIC"
)

add_dependencies(points2vscan
  points2vscan_ui_mainwindow
  points2vscan_mainwindow_moc
)

target_link_libraries(points2vscan
  ${catkin_LIBRARIES}
  rosinterface
  nlopt
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

# points2image
add_executable(points2image nodes/points2image/points2image.cpp)
target_link_libraries(points2image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  points_image
)
add_dependencies(points2image
  points_image
  points2image_generate_messages_cpp
  calibration_camera_lidar_generate_messages_cpp
)

# vscan2image
add_executable(vscan2image nodes/vscan2image/vscan2image.cpp)
target_link_libraries(vscan2image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  points_image
)
add_dependencies(vscan2image
  points_image
  points2image_generate_messages_cpp
  calibration_camera_lidar_generate_messages_cpp
)


# vscan2linelist
add_executable(vscan2linelist nodes/vscan2linelist/vscan2linelist.cpp)
target_link_libraries(vscan2linelist
  ${catkin_LIBRARIES}
)
