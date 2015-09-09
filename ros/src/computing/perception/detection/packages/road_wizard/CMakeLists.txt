cmake_minimum_required(VERSION 2.8.3)
project(road_wizard)

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
  ValueSet.msg
  ColorSet.msg
  TunedResult.msg
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

### region_tlr ###
add_executable(region_tlr
  nodes/region_tlr/region_tlr.cpp
  nodes/region_tlr/TrafficLightDetector.cpp
  nodes/region_tlr/Context.cpp
  )

target_link_libraries(region_tlr
  ${catkin_LIBRARIES}
)

add_dependencies(region_tlr
runtime_manager_generate_messages_cpp
road_wizard_generate_messages_cpp)

### feat_proj ###
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${libvectormap_INCLUDE_DIRS}
)

add_executable(feat_proj
  nodes/feat_proj/feat_proj.cpp
)

target_link_libraries(feat_proj
  ${catkin_LIBRARIES}
  ${libvectormap_LIBRARIES}
)

add_dependencies(feat_proj
runtime_manager_generate_messages_cpp
road_wizard_generate_messages_cpp
)

### tlr_tuner ###
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
  noddes/tlr_tuner/
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
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tuner/ui_mainwindow.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tuner/mainwindow.ui"
)
add_custom_target(tlr_tuner_ui_mainwindow DEPENDS ui_mainwindow.h)

add_custom_command(
  OUTPUT mainwindow_moc.cpp
  COMMAND
  ${Qt5BIN}/moc
  "-o" "${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tuner/mainwindow_moc.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tuner/mainwindow.h"
)
add_custom_target(tlr_tuner_mainwindow_moc DEPENDS mainwindow_moc.cpp)

add_executable(tlr_tuner
  nodes/tlr_tuner/tlr_tuner.cpp
  nodes/tlr_tuner/mainwindow.cpp
  nodes/tlr_tuner/tunerBody.cpp
  )

set_target_properties(tlr_tuner
  PROPERTIES COMPILE_FLAGS "-fPIC"
  )

add_dependencies(tlr_tuner
  tlr_tuner_ui_mainwindow
  tlr_tuner_mainwindow_moc
  )

target_link_libraries(tlr_tuner
  ${catkin_LIBRARIES}
  ${Qt5LIBROOT}/libQt5Core.so
  ${Qt5LIBROOT}/libQt5Gui.so
  ${Qt5LIBROOT}/libQt5OpenGL.so
  ${Qt5LIBROOT}/libQt5Widgets.so
  )