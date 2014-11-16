cmake_minimum_required(VERSION 2.8.3)
project(car_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

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
  LIBRARIES dpm dpm_gpu
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/opencv2/legacy)
include_directories(/usr/local/cuda/include)

add_executable(car_detector
  nodes/car_detector/car_detector.cpp
)

SET_TARGET_PROPERTIES(car_detector
  PROPERTIES
  COMPILE_FLAGS "-DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin"
)

target_link_libraries(car_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  cuda
  dpm
  dpm_gpu
)