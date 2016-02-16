cmake_minimum_required(VERSION 2.8.3)
project(cv_tracker)

include(FindPkgConfig)
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  libdpm_ocv
  libdpm_ttic
  fusion
  kf
  tf
  runtime_manager
  calibration_camera_lidar
)
FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

pkg_check_modules(HDF5 hdf5)

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_message_files(
  FILES
  image_obj.msg
  image_obj_ranged.msg
  image_obj_tracked.msg
  image_rect.msg
  image_rect_ranged.msg
  obj_label.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS libdpm_ocv libdpm_ttic scan2image fusion kf runtime_manager calibration_camera_lidar
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -DROS ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
  lib
)

IF(CUDA_FOUND)
INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

IF("${ARCHITECTURE}" MATCHES "^arm")
LINK_DIRECTORIES(/usr/lib/arm-linux-gnueabihf/tegra)
endif()

ELSE()
message("CUDA is not installed. 'cv_tracker' package is not built.")
ENDIF()

## dpm_ttic ##
ADD_EXECUTABLE(dpm_ttic
  nodes/dpm_ttic/dpm_ttic.cpp
)

TARGET_LINK_LIBRARIES(dpm_ttic
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  libdpm_ttic
)

IF(CUDA_FOUND)
SET_TARGET_PROPERTIES(dpm_ttic
  PROPERTIES
  COMPILE_FLAGS
  "-DHAS_GPU -DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin -DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
)
ELSE()
SET_TARGET_PROPERTIES(dpm_ttic
  PROPERTIES
  COMPILE_FLAGS
  "-DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
)
ENDIF()


ADD_DEPENDENCIES(dpm_ttic
  cv_tracker_generate_messages_cpp
  runtime_manager_generate_messages_cpp
)

## dpm_ocv
INCLUDE_DIRECTORIES(../../lib/image/dpm_ocv/src)

ADD_EXECUTABLE(dpm_ocv
  nodes/dpm_ocv/dpm_ocv.cpp
)

TARGET_LINK_LIBRARIES(dpm_ocv
  libdpm_ocv
  ${catkin_LIBRARIES}
  #${OpenCV_LIBS}
)

ADD_DEPENDENCIES(dpm_ocv
  runtime_manager_generate_messages_cpp
)

IF(CUDA_FOUND)
SET_TARGET_PROPERTIES(dpm_ocv
  PROPERTIES
  COMPILE_FLAGS
  "-DHAS_GPU -DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/gpu_function.cubin -DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/model/models_VOC2007/"
)
ELSE()
SET_TARGET_PROPERTIES(dpm_ocv
  PROPERTIES
  COMPILE_FLAGS
  "-DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
)
ENDIF()

#SET_TARGET_PROPERTIES(car_locate
#  PROPERTIES COMPILE_FLAGS
#  "-DCAMERA_YAML=${CAMERA_YAML}")

## range_fusion ##
ADD_EXECUTABLE(range_fusion
  nodes/range_fusion/range_fusion.cpp
)

TARGET_LINK_LIBRARIES(range_fusion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  fusion
)

ADD_DEPENDENCIES(range_fusion
  cv_tracker_generate_messages_cpp
  runtime_manager_generate_messages_cpp
)

## kf_tracker ##
ADD_EXECUTABLE(kf_track
  nodes/kf_track/kf_track.cpp
)

TARGET_LINK_LIBRARIES(kf_track
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  kf
)

## obj_reproj ##
ADD_EXECUTABLE(obj_reproj
  nodes/obj_reproj/obj_reproj.cpp
)

TARGET_LINK_LIBRARIES(obj_reproj
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

ADD_DEPENDENCIES(obj_reproj
  cv_tracker_generate_messages_cpp
  calibration_camera_lidar_generate_messages_cpp
)

## klt_tracker ##
ADD_EXECUTABLE(klt_track
  nodes/klt_track/klt_track.cpp
)

INCLUDE_DIRECTORIES(lib/lktracker)

ADD_LIBRARY(lktracker
  lib/lktracker/LkTracker.cpp
)
TARGET_LINK_LIBRARIES(lktracker
  ${OpenCV_LIBS}
)
ADD_DEPENDENCIES(lktracker
  cv_tracker_generate_messages_cpp
)

TARGET_LINK_LIBRARIES(klt_track
  lktracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

ADD_DEPENDENCIES(klt_track
  libdpm_ocv_generate_messages_cpp
)

############# RCNN#############

###########################################CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
##############DONT FORGET TO INSTALL fastrcnn's caffe dynamic libraries to /usr/local/lib	 	### 
set(CAFFE_PATH "$ENV{HOME}/fast-rcnn/caffe-fast-rcnn/distribute")
####################################################################################################

if(EXISTS "${CAFFE_PATH}")
  find_package(librcnn)

  INCLUDE_DIRECTORIES(
    ${librcnn_INCLUDE_DIRS}
    ${CAFFE_PATH}/include
  )
IF(HDF5_FOUND)
  INCLUDE_DIRECTORIES(${HDF5_INCLUDE_DIRS})
ENDIF()

  ADD_EXECUTABLE(rcnn_node
    nodes/rcnn/rcnn_node.cpp
  )

  TARGET_LINK_LIBRARIES(rcnn_node
    librcnn
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  )

  ADD_DEPENDENCIES(rcnn_node
    cv_tracker_generate_messages_cpp
    runtime_manager_generate_messages_cpp
    librcnn
  )
else()
  message("'FastRCNN/Caffe' are not installed. 'rcnn_node' will not be built.")
endif()
