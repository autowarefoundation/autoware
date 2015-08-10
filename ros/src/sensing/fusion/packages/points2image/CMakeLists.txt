cmake_minimum_required(VERSION 2.8.3)
project(points2image)

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
)

find_package(OpenCV REQUIRED)

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
   CATKIN_DEPENDS rosinterface
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result ${CMAKE_CXX_FLAGS}")

if(EXISTS /usr/local/qtbase5)
# Self install version
set(Qt5INCLUDE "/usr/local/qtbase5/include")
set(Qt5BIN "/usr/local/qtbase5/bin")
set(Qt5LIBROOT "/usr/local/qtbase5/lib")
else()
# Ubuntu Qt5 package
set(Qt5INCLUDE "/usr/include/qt5")
set(Qt5BIN "/usr/lib/x86_64-linux-gnu/qt5/bin")
set(Qt5LIBROOT "/usr/lib/x86_64-linux-gnu")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${Qt5INCLUDE}
  ${Qt5INCLUDE}/QtCore
  ${Qt5INCLUDE}/QtGui
  ${Qt5INCLUDE}/QtOpenGL
  ${Qt5INCLUDE}/QtWidgets
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
  ${Qt5LIBROOT}/libQt5Core.so
  ${Qt5LIBROOT}/libQt5Gui.so
  ${Qt5LIBROOT}/libQt5OpenGL.so
  ${Qt5LIBROOT}/libQt5Widgets.so
  nlopt
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
  point2image_generate_messages_cpp
)


# vscan2linelist
add_executable(vscan2linelist nodes/vscan2linelist/vscan2linelist.cpp)
target_link_libraries(vscan2linelist
  ${catkin_LIBRARIES}
)
