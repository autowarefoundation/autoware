cmake_minimum_required(VERSION 2.8.12)
project(cnn_lidar_detector)

####################################
#####ROS VERSION CHECKER############
####################################
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if (CMAKE_VERSION VERSION_LESS "3.1")
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()
####################################

include(FindPkgConfig)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  jsk_recognition_msgs
  jsk_rviz_plugins
)

FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(ZLIB REQUIRED)

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

catkin_package(CATKIN_DEPENDS 
  message_runtime
  std_msgs
  pcl_ros
)
###########################################
###_CUDA COMPUTING CAPABILITY DETECTION####
###########################################
IF(CUDA_FOUND)
  if(NOT CUDA_CAPABILITY_VERSION_CHECKER)
    set(CUDA_CAPABILITY_VERSION_CHECKER "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker")
  endif()

  execute_process(
    COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
    OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

  if("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
    set(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
  else()
    set(CUDA_ARCH "sm_52")
  endif()
ENDIF()
############################################


##################
## Build Node(s)##
##################

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
)

####### MatlabIO Library #######
add_library(matlab_io SHARED
  src/MatlabIO.cpp
)

target_include_directories(matlab_io PRIVATE
  ${OPENCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ZLIB_INCLUDE_DIR}
  include
)

target_link_libraries(matlab_io 
  ${Boost_LIBRARIES} 
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}
)

#####CNN LIDAR########
##############################SSD'sFORK of CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
set(CNN_CAFFE_PATH "$ENV{HOME}/ssdcaffe/distribute")
####################################################################################################
if(EXISTS "${CNN_CAFFE_PATH}")

  ADD_EXECUTABLE(cnn_lidar_detector_node
    src/cnn_lidar_detector_node.cpp
    src/cnn_lidar_detector.cpp
  )

  TARGET_LINK_LIBRARIES(cnn_lidar_detector_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CNN_CAFFE_PATH}/lib/libcaffe.so
    glog
    matlab_io
  )

  TARGET_INCLUDE_DIRECTORIES( cnn_lidar_detector_node PRIVATE
    ${CUDA_INCLUDE_DIRS}
    ${CNN_CAFFE_PATH}/include
    include
  )
else()
  message("'SSD/Caffe' is not installed. 'cnn_lidar_detector' will not be built.")
endif()
