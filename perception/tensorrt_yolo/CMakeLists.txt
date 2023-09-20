cmake_minimum_required(VERSION 3.14)
project(tensorrt_yolo)

find_package(autoware_cmake REQUIRED)
autoware_package()

# TODO(tensorrt_yolo): Remove once upgrading to TensorRT 8.5 is complete
add_compile_options(-Wno-deprecated-declarations)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

find_package(OpenCV REQUIRED)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message(STATUS "CUDA is available!")
    message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVONNXPARSER nvonnxparser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  if(CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVPARSERS: ${NVPARSERS}")
    message(STATUS "NVINFER_PLUGIN: ${NVINFER_PLUGIN}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
PATH_SUFFIXES lib lib64 bin
DOC "CUDNN library."
)
if(CUDNN_LIBRARY)
  if(CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# create calib image directory for int8 calibration
set(CALIB_IMAGE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/calib_image")
if(NOT EXISTS "${CALIB_IMAGE_PATH}")
  execute_process(COMMAND mkdir -p ${CALIB_IMAGE_PATH})
endif()

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  include_directories(
    include
    lib/include
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )


  ### yolo ###
  cuda_add_library(mish_plugin SHARED
    lib/src/plugins/mish.cu
    lib/src/plugins/mish_plugin.cpp
  )

  cuda_add_library(yolo_layer_plugin SHARED
    lib/src/plugins/yolo_layer.cu
    lib/src/plugins/yolo_layer_plugin.cpp
  )

  cuda_add_library(nms_plugin SHARED
    lib/src/plugins/nms.cu
    lib/src/plugins/nms_plugin.cpp
  )

  ament_auto_add_library(yolo SHARED
    lib/src/trt_yolo.cpp
  )

  target_include_directories(yolo PUBLIC
    lib/include
  )

  target_link_libraries(yolo
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
    mish_plugin
    yolo_layer_plugin
    nms_plugin
  )

  ament_auto_add_library(tensorrt_yolo_nodelet SHARED
    src/nodelet.cpp
  )

  target_link_libraries(tensorrt_yolo_nodelet
    ${OpenCV_LIBS}
    yolo
    mish_plugin
    yolo_layer_plugin
    nms_plugin
  )

  rclcpp_components_register_node(tensorrt_yolo_nodelet
    PLUGIN "object_recognition::TensorrtYoloNodelet"
    EXECUTABLE tensorrt_yolo_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    config
    launch
  )

  install(
    TARGETS
      mish_plugin
      yolo_layer_plugin
      nms_plugin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

else()
  message("TensorrtYolo won't be built, CUDA and/or TensorRT were not found.")
  ament_auto_package(INSTALL_TO_SHARE
    config
    launch
  )
endif()
