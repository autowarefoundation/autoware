cmake_minimum_required(VERSION 2.8.3)
project(road_wizard)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include(FindPkgConfig)
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
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

pkg_check_modules(Qt5Core REQUIRED Qt5Core)
pkg_check_modules(Qt5Widgets REQUIRED Qt5Widgets)

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
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS std_msgs message_runtime libvectormap runtime_manager
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

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
  ${OpenCV_LIBS}
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
  ${OpenCV_LIBS}
)

add_dependencies(feat_proj
runtime_manager_generate_messages_cpp
road_wizard_generate_messages_cpp
)

EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  nodes/tlr_tuner/
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
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
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${OpenCV_LIBS}
  )
