cmake_minimum_required(VERSION 2.8.3)
project(librcnn)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)
FIND_PACKAGE(CUDA)
find_package(OpenCV REQUIRED)

pkg_check_modules(HDF5 hdf5)

###########################################CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
##############DONT FORGET TO INSTALL fastrcnn's caffe dynamic libraries to /usr/local/lib	#### 
set(CAFFE_PATH "$ENV{HOME}/fast-rcnn/caffe-fast-rcnn/distribute")				####
####################################################################################################

if(EXISTS "${CAFFE_PATH}")

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

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(
  include
  ${catkin_INCLUDE_DIRS}
  ${CAFFE_PATH}/include
)
IF(CUDA_FOUND)
INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})
ENDIF()
IF(HDF5_FOUND)
INCLUDE_DIRECTORIES(${HDF5_INCLUDE_DIRS})
ENDIF()

## Declare a cpp library
add_library(librcnn
   src/rcnn_detector.cpp
)

target_link_libraries(librcnn
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  glog
  ${CAFFE_PATH}/lib/libcaffe.so
)

#############
## Install ##
#############

INSTALL(
  DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

else()
  message("'FastRCNN/Caffe ' are not installed. 'librcnn' will not be built.")
endif()
