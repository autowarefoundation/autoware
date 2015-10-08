cmake_minimum_required(VERSION 2.8.3)
project(librcnn)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

###########################################CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
##############DONT FORGET TO INSTALL fastrcnn's caffe dynamic libraries to /usr/local/lib	#### 
set(CAFFE_PATH "$ENV{HOME}/fast-rcnn/caffe-fast-rcnn/distribute")				####
set(CUDA_INCLUDE_PATH "/usr/local/cuda/include/")						####
####################################################################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES librcnn
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(src)
include_directories(${CAFFE_PATH}/include)
include_directories(${CUDA_INCLUDE_PATH})

## Declare a cpp library
add_library(librcnn
   src/rcnn_detector.cpp
)

target_link_libraries(librcnn
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  glog
  caffe
)

add_dependencies(librcnn runtime_manager_generate_messages_cpp cv_tracker_generate_messages_cpp)

#############
## Install ##
#############

INSTALL(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
