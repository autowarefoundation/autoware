cmake_minimum_required(VERSION 2.8.12)
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
  CATKIN_DEPENDS libdpm_ocv libdpm_ttic scan2image fusion kf runtime_manager calibration_camera_lidar message_runtime std_msgs geometry_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

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

## dummy_tracker ##
ADD_EXECUTABLE(dummy_track
  nodes/dummy_track/dummy_track.cpp
)
TARGET_LINK_LIBRARIES(dummy_track
  ${catkin_LIBRARIES}
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

# check whether jsk library is available or not
find_package(jsk_recognition_msgs)
if (jsk_recognition_msgs_FOUND)
  # if jsk plugin is found, activate macro
  SET_TARGET_PROPERTIES(obj_reproj
    PROPERTIES COMPILE_FLAGS
    "-DHAVE_JSK_PLUGIN")
endif (jsk_recognition_msgs_FOUND)

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
set(RCNN_CAFFE_PATH "$ENV{HOME}/fast-rcnn/caffe-fast-rcnn/distribute")
####################################################################################################

if(EXISTS "${RCNN_CAFFE_PATH}")
  find_package(librcnn)
#  INCLUDE_DIRECTORIES(
#    ${librcnn_INCLUDE_DIRS}
#    ${RCNN_CAFFE_PATH}/include
#  )

IF(HDF5_FOUND)
  INCLUDE_DIRECTORIES(${HDF5_INCLUDE_DIRS})
ENDIF()

  ADD_EXECUTABLE(rcnn_msr
    nodes/rcnn/rcnn_node.cpp
  )

  TARGET_LINK_LIBRARIES(rcnn_msr
    librcnn
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  )

  TARGET_INCLUDE_DIRECTORIES (rcnn_msr PRIVATE
    ${librcnn_INCLUDE_DIRS}
    ${RCNN_CAFFE_PATH}/include
  )

  ADD_DEPENDENCIES(rcnn_msr
    cv_tracker_generate_messages_cpp
    runtime_manager_generate_messages_cpp
    librcnn
  )
else()
  message("'FastRCNN/Caffe' are not installed. 'rcnn_msr' will not be built.")
endif()

#####SSD########
##############################SSD'sFORK of CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
set(SSD_CAFFE_PATH "$ENV{HOME}/ssdcaffe/distribute")
####################################################################################################
if(EXISTS "${SSD_CAFFE_PATH}")

  IF(HDF5_FOUND)
    INCLUDE_DIRECTORIES(${HDF5_INCLUDE_DIRS})
  ENDIF()

  ADD_EXECUTABLE(ssd_unc
    nodes/ssd/ssd_node.cpp
    nodes/ssd/ssd_detector.cpp
  )

  TARGET_LINK_LIBRARIES(ssd_unc
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${SSD_CAFFE_PATH}/lib/libcaffe.so
    glog
  )

  TARGET_INCLUDE_DIRECTORIES( ssd_unc PRIVATE
    ${SSD_CAFFE_PATH}/include
    nodes/ssd/include
  )

  ADD_DEPENDENCIES(ssd_unc
    cv_tracker_generate_messages_cpp
    runtime_manager_generate_messages_cpp
  )
else()
  message("'SSD/Caffe' is not installed. 'ssd_unc' will not be built.")
endif()

###YOLO2####
###########YOLO2 PATH#####################################
set(YOLO2_PATH "$ENV{HOME}/darknet")
##########################################################
if(EXISTS "${YOLO2_PATH}")

  set(YOLO2_USE_CUDNN 1)
  set(YOLO2_USE_GPU 1)
  
  list(APPEND CUDA_NVCC_FLAGS "-std=c++11")

  cuda_add_library(yolo2lib_wa
    ${YOLO2_PATH}/src/activation_kernels.cu
    ${YOLO2_PATH}/src/activation_layer.c
    ${YOLO2_PATH}/src/activations.c
    ${YOLO2_PATH}/src/avgpool_layer.c
    ${YOLO2_PATH}/src/avgpool_layer_kernels.cu
    ${YOLO2_PATH}/src/batchnorm_layer.c
    ${YOLO2_PATH}/src/blas.c
    ${YOLO2_PATH}/src/blas_kernels.cu
    ${YOLO2_PATH}/src/box.c
    ${YOLO2_PATH}/src/col2im.c
    ${YOLO2_PATH}/src/col2im_kernels.cu
    ${YOLO2_PATH}/src/connected_layer.c
    ${YOLO2_PATH}/src/convolutional_kernels.cu
    ${YOLO2_PATH}/src/convolutional_layer.c
    ${YOLO2_PATH}/src/cost_layer.c
    ${YOLO2_PATH}/src/crnn_layer.c
    ${YOLO2_PATH}/src/crop_layer.c
    ${YOLO2_PATH}/src/crop_layer_kernels.cu
    ${YOLO2_PATH}/src/cuda.c
    ${YOLO2_PATH}/src/data.c
    ${YOLO2_PATH}/src/detection_layer.c
    ${YOLO2_PATH}/src/dropout_layer.c
    ${YOLO2_PATH}/src/dropout_layer_kernels.cu
    ${YOLO2_PATH}/src/gemm.c
    ${YOLO2_PATH}/src/gru_layer.c
    ${YOLO2_PATH}/src/im2col.c
    ${YOLO2_PATH}/src/im2col_kernels.cu
    ${YOLO2_PATH}/src/image.c
    ${YOLO2_PATH}/src/layer.c
    ${YOLO2_PATH}/src/list.c
    ${YOLO2_PATH}/src/local_layer.c
    ${YOLO2_PATH}/src/matrix.c
    ${YOLO2_PATH}/src/maxpool_layer.c
    ${YOLO2_PATH}/src/maxpool_layer_kernels.cu
    ${YOLO2_PATH}/src/network.c
    ${YOLO2_PATH}/src/network_kernels.cu
    ${YOLO2_PATH}/src/normalization_layer.c
    ${YOLO2_PATH}/src/option_list.c
    ${YOLO2_PATH}/src/parser.c
    ${YOLO2_PATH}/src/region_layer.c
    ${YOLO2_PATH}/src/reorg_layer.c
    ${YOLO2_PATH}/src/rnn_layer.c
    ${YOLO2_PATH}/src/route_layer.c
    ${YOLO2_PATH}/src/shortcut_layer.c
    ${YOLO2_PATH}/src/softmax_layer.c
    ${YOLO2_PATH}/src/tree.c
    ${YOLO2_PATH}/src/utils.c

    OPTIONS 
    -DGPU=${YOLO2_USE_GPU}
    -DCUDNN=${YOLO2_USE_CUDNN}
  )

  cuda_add_cublas_to_target(yolo2lib_wa)

  target_include_directories(yolo2lib_wa PRIVATE
    ${YOLO2_PATH}/src
  )

  target_link_libraries(yolo2lib_wa
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  )

  add_executable(yolo2_wa
    nodes/yolo2/src/yolo2_node.cpp
    nodes/yolo2/src/darknet/yolo2.cpp
  )
  target_include_directories(yolo2_wa PRIVATE
    nodes/yolo2/include
    ${YOLO2_PATH}/src
  )

  target_link_libraries(yolo2_wa
    yolo2lib_wa
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  )

else()
  message("'Darknet' was not found. 'yolo2_wa' will not be built.")
endif()
