cmake_minimum_required(VERSION 2.8.3)
project(calibration_camera_lidar)

include(FindPkgConfig)
include(FindLibXml2)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  sensor_msgs
  glviewer
  rosinterface
  message_generation
  cv_bridge
  pcl_ros
  image_transport
)

find_package(OpenCV REQUIRED)

pkg_check_modules(Qt5Core REQUIRED Qt5Core)
pkg_check_modules(Qt5Widgets REQUIRED Qt5Widgets)
pkg_check_modules(Qt5OpenGL REQUIRED Qt5OpenGL)

add_message_files(
  FILES
  projection_matrix.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera
   CATKIN_DEPENDS glviewer rosinterface
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -g -Wno-unused-result ${CMAKE_CXX_FLAGS}")

EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  CalibrationToolkit
  nodes/calibration_toolkit
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
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
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
)

add_custom_command(
  OUTPUT ui_mainwindow.h
  COMMAND
  ${Qt5BIN}/uic
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/calibration_toolkit/ui_mainwindow.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/calibration_toolkit/mainwindow.ui"
)
add_custom_target(ui_mainwindow DEPENDS ui_mainwindow.h)

add_custom_command(
  OUTPUT mainwindow_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/calibration_toolkit/mainwindow_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/calibration_toolkit/mainwindow.h"
)
add_custom_target(mainwindow_moc DEPENDS mainwindow_moc.cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBXML2_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}

  CalibrationToolkit
  nodes/calibration_toolkit
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
)

## 2D
add_executable(calibration_test
  nodes/calibration_test/chessboard.cpp
  nodes/calibration_test/trans.cpp
  nodes/calibration_test/scan_window.cpp
  nodes/calibration_test/common_2d_calib.cpp
  nodes/calibration_test/image_window.cpp
  nodes/calibration_test/camera_lidar2d_offline_calib.cpp)

set(PARAM_YAML "${CMAKE_CURRENT_SOURCE_DIR}/param.yaml")
set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

set_target_properties(calibration_test
  PROPERTIES COMPILE_FLAGS
  "-DPARAM_YAML=${PARAM_YAML} -DCAMERA_YAML=${CAMERA_YAML}")

target_link_libraries(calibration_test
   ${catkin_LIBRARIES} ${OpenCV_LIBS} xml2)

## 3D
add_executable(calibration_toolkit
  nodes/calibration_toolkit/main.cpp
  nodes/calibration_toolkit/mainwindow.cpp
)

set_target_properties(calibration_toolkit
  PROPERTIES COMPILE_FLAGS "-fPIC"
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(calibration_toolkit
  ui_mainwindow
  mainwindow_moc
)

## Specify libraries to link a library or executable target against
target_link_libraries(calibration_toolkit
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  glviewer
  rosinterface
  calibrationtoolkit
  glut
  GLU
  nlopt
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
)

# calibration_publisher
add_executable(calibration_publisher
  nodes/calibration_publisher/calibration_publisher.cpp
  )

add_dependencies(calibration_publisher
  calibration_camera_lidar_generate_messages_cpp
)

target_link_libraries(calibration_publisher
  ${catkin_LIBRARIES}
  )
