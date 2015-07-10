cmake_minimum_required(VERSION 2.8.3)
project(dpm_ttic)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES dpm_ttic
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(include/dpm_ttic ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(src)

file(GLOB DPM_TTIC_OCV_CXX_SOURCES "src/*.cpp")

ADD_LIBRARY(dpm_ttic
  ${DPM_TTIC_OCV_CXX_SOURCES}
)

TARGET_LINK_LIBRARIES(dpm_ttic
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#############
## Install ##
#############

INSTALL(
  DIRECTORY include/dpm_ttic
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
