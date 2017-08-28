cmake_minimum_required(VERSION 2.8.12)
project(image_segmenter)
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

  ADD_EXECUTABLE(image_segmenter_enet
    src/image_segmenter_enet_node.cpp
    src/image_segmenter_enet.cpp
  )
	
  TARGET_LINK_LIBRARIES(image_segmenter_enet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${ENET_CAFFE_PATH}/lib/libcaffe.so
    glog
  )

  TARGET_INCLUDE_DIRECTORIES(image_segmenter_enet PRIVATE
    ${CUDA_INCLUDE_DIRS}
    ${ENET_CAFFE_PATH}/include
    include
  )

  ADD_DEPENDENCIES(image_segmenter_enet
      ${catkin_EXPORTED_TARGETS}
  )
else()
  message("'ENet/Caffe' is not installed. 'image_segmenter_enet' will not be built.")
endif()