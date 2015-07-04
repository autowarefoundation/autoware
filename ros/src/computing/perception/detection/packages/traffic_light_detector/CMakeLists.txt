cmake_minimum_required(VERSION 2.8.3)
project(traffic_light_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_generation
  sensor_msgs
  tf
  libvectormap
  runtime_manager
  )
find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)

## Generate added messages and services with any dependencies listed here
add_message_files(FILES
  ExtractedPosition.msg
  Signals.msg
  )

generate_messages(DEPENDENCIES
  sensor_msgs
  std_msgs
  )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime libvectormap runtime_manager
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########

### traffic_light_lkf ###
add_executable(traffic_light_lkf
  nodes/traffic_light_lkf/traffic_light_lkf.cpp
  nodes/traffic_light_lkf/TrafficLightDetector.cpp
  nodes/traffic_light_lkf/Context.cpp
  )

target_link_libraries(traffic_light_lkf
  ${catkin_LIBRARIES}
)

add_dependencies(traffic_light_lkf 
runtime_manager_generate_messages_cpp
traffic_light_detector_generate_messages_cpp)

### traffic_light_extract ###
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${libvectormap_INCLUDE_DIRS}
)

add_executable(traffic_light_extract
  nodes/traffic_light_extract/traffic_light_extract.cpp
)

target_link_libraries(traffic_light_extract
  ${catkin_LIBRARIES}
  ${libvectormap_LIBRARIES}
)

add_dependencies(traffic_light_extract
traffic_light_detector_generate_messages_cpp
)

### traffic_light_detector_tuner ###
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
  noddes/traffic_light_detector_tuner/
  ${Qt5INCLUDE}
  ${Qt5INCLUDE}/QtCore
  ${Qt5INCLUDE}/QtGui
  ${Qt5INCLUDE}/QtOpenGL
  ${Qt5INCLUDE}/QtWidgets
)

add_custom_command(
  OUTPUT ui_mainwindow.h
  COMMAND
  ${Qt5BIN}/uic
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/traffic_light_detector_tuner/ui_mainwindow.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/traffic_light_detector_tuner/mainwindow.ui"
)
add_custom_target(traffic_light_detector_tuner_ui_mainwindow DEPENDS ui_mainwindow.h)

add_custom_command(
  OUTPUT mainwindow_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/traffic_light_detector_tuner/mainwindow_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/traffic_light_detector_tuner/mainwindow.h"
)
add_custom_target(traffic_light_detector_tuner_mainwindow_moc DEPENDS mainwindow_moc.cpp)

add_executable(traffic_light_detector_tuner
  nodes/traffic_light_detector_tuner/traffic_light_detector_tuner.cpp
  nodes/traffic_light_detector_tuner/mainwindow.cpp
  nodes/traffic_light_detector_tuner/tunerBody.cpp
  )

set_target_properties(traffic_light_detector_tuner
  PROPERTIES COMPILE_FLAGS "-fPIC"
  )

add_dependencies(traffic_light_detector_tuner
  traffic_light_detector_tuner_ui_mainwindow
  traffic_light_detector_tuner_mainwindow_moc
  )

target_link_libraries(traffic_light_detector_tuner
  ${catkin_LIBRARIES}
  ${Qt5LIBROOT}/libQt5Core.so
  ${Qt5LIBROOT}/libQt5Gui.so
  ${Qt5LIBROOT}/libQt5OpenGL.so
  ${Qt5LIBROOT}/libQt5Widgets.so
  )