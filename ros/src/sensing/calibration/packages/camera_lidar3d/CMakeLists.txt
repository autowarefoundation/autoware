cmake_minimum_required(VERSION 2.8.3)
project(camera_lidar3d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  sensor_msgs
  glviewer
  rosinterface
  pcl_ros
)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_lidar3d
   CATKIN_DEPENDS glviewer rosinterface
#  DEPENDS system_lib
)

###########
## Build ##
###########

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

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  CalibrationToolkit
  nodes/camera_lidar3d
  ${Qt5INCLUDE}
  ${Qt5INCLUDE}/QtCore
  ${Qt5INCLUDE}/QtGui
  ${Qt5INCLUDE}/QtOpenGL
  ${Qt5INCLUDE}/QtWidgets
)

add_custom_command(
  OUTPUT calibrationtoolkit_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/CalibrationToolkit/calibrationtoolkit_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/CalibrationToolkit/calibrationtoolkit.h"
)
add_custom_target(calibrationtoolkit_moc
  DEPENDS calibrationtoolkit_moc.cpp
)

add_custom_command(
  OUTPUT selectionwidget_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/CalibrationToolkit/selectionwidget_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/CalibrationToolkit/selectionwidget.h"
)
add_custom_target(selectionwidget_moc
  DEPENDS selectionwidget_moc.cpp
)

## Declare a cpp library
add_library(calibrationtoolkit
  CalibrationToolkit/calibrationtoolkit.cpp
  CalibrationToolkit/selectionwidget.cpp
)
add_dependencies(calibrationtoolkit
  glviewer
  calibrationtoolkit_moc
  selectionwidget_moc
)
target_link_libraries(calibrationtoolkit
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  glviewer
  rosinterface
  nlopt
  ${Qt5LIBROOT}/libQt5OpenGL.so
)

add_custom_command(
  OUTPUT ui_mainwindow.h
  COMMAND
  ${Qt5BIN}/uic
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/camera_lidar3d_offline_calib/ui_mainwindow.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/camera_lidar3d_offline_calib/mainwindow.ui"
)
add_custom_target(ui_mainwindow DEPENDS ui_mainwindow.h)

add_custom_command(
  OUTPUT mainwindow_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/camera_lidar3d_offline_calib/mainwindow_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/camera_lidar3d_offline_calib/mainwindow.h"
)
add_custom_target(mainwindow_moc DEPENDS mainwindow_moc.cpp)

## Declare a cpp executable
add_executable(camera_lidar3d_offline_calib
  nodes/camera_lidar3d_offline_calib/main.cpp
  nodes/camera_lidar3d_offline_calib/mainwindow.cpp
)

set_target_properties(camera_lidar3d_offline_calib
  PROPERTIES COMPILE_FLAGS "-fPIC"
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(camera_lidar3d_offline_calib
  ui_mainwindow
  mainwindow_moc
)

## Specify libraries to link a library or executable target against
target_link_libraries(camera_lidar3d_offline_calib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  glviewer
  rosinterface
  calibrationtoolkit
  glut
  GLU
  ${Qt5LIBROOT}/libQt5Core.so
  ${Qt5LIBROOT}/libQt5Gui.so
  ${Qt5LIBROOT}/libQt5OpenGL.so
  ${Qt5LIBROOT}/libQt5Widgets.so
  nlopt
)
