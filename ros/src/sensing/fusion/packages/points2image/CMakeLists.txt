cmake_minimum_required(VERSION 2.8.3)
project(points2image)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  autoware_msgs
  rosinterface
  fastvirtualscan
  cv_bridge
  tf
  pcl_ros
  pcl_conversions
)

find_package(OpenCV REQUIRED)

set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_lidar3d
   CATKIN_DEPENDS rosinterface message_runtime std_msgs sensor_msgs autoware_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

EXECUTE_PROCESS(
  COMMAND pkg-config --variable=host_bins Qt5Core
  OUTPUT_VARIABLE Qt5BIN
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

# library
add_library(points_image
  lib/points_image/points_image.cpp
)
add_dependencies(points_image autoware_msgs_generate_messages_cpp)

qt5_wrap_ui(points2vscan_ui_mainwindow nodes/points2vscan/mainwindow.ui)

# points2vscan
add_executable(points2vscan
  nodes/points2vscan/main.cpp
  nodes/points2vscan/mainwindow.cpp
  nodes/points2vscan/mainwindow.h
  ${points2vscan_ui_mainwindow}
)

set_target_properties(points2vscan
  PROPERTIES COMPILE_FLAGS "-fPIC"
)

target_link_libraries(points2vscan
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
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
  autoware_msgs_generate_messages_cpp
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
  autoware_msgs_generate_messages_cpp
)


# vscan2linelist
add_executable(vscan2linelist nodes/vscan2linelist/vscan2linelist.cpp)
target_link_libraries(vscan2linelist
  ${catkin_LIBRARIES}
)
