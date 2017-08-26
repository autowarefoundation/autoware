cmake_minimum_required(VERSION 2.8.12)
project(enet_image_segmenter)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

include(FindPkgConfig)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  autoware_msgs
)
elseif ("${ROS_VERSION}" MATCHES "(kinetic)")
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  autoware_msgs
  sensor_msgs
)
endif()

FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs autoware_msgs
)
elseif ("${ROS_VERSION}" MATCHES "(kinetic)")
catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs autoware_msgs
)
endif()
###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
)


#####ENET########
##############################ENet's CAFFE FORK NEEDS TO BE PREVIOUSLY COMPILED####################
set(ENET_CAFFE_PATH "$ENV{HOME}/ENet/caffe-enet/distribute")
####################################################################################################
if(EXISTS "${ENET_CAFFE_PATH}")

  ADD_EXECUTABLE(enet_image_segmenter
    src/enet_image_segmenter_node.cpp
    src/enet_image_segmenter.cpp
  )
	
  TARGET_LINK_LIBRARIES(enet_image_segmenter
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${ENET_CAFFE_PATH}/lib/libcaffe.so
    glog
  )

  TARGET_INCLUDE_DIRECTORIES(enet_image_segmenter PRIVATE
    ${CUDA_INCLUDE_DIRS}
    ${ENET_CAFFE_PATH}/include
    include
  )

  ADD_DEPENDENCIES(enet_image_segmenter
      ${catkin_EXPORTED_TARGETS}
  )
else()
  message("'ENet/Caffe' is not installed. 'enet_image_segmenter' will not be built.")
endif()