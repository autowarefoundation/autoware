cmake_minimum_required(VERSION 3.14)
project(traffic_light_fine_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

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
  message(STATUS "CUDA NOT FOUND")
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
  message(STATUS "TensorRT is NOT Available")
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
  message(STATUS "CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# Download caffemodel and prototxt
set(PRETRAINED_MODEL_BATCH_1_HASH 2b72d085022b8ee6aacff06bd722cfda)
set(PRETRAINED_MODEL_BATCH_4_HASH 4044daa86e7776ce241e94d98a09cc0e)
set(PRETRAINED_MODEL_BATCH_6_HASH 47255a11bde479320d703f1f45db1242)
set(LAMP_LABEL_HASH e9f45efb02f2a9aa8ac27b3d5c164905)
set(DATA_DIR https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2)

set(DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if(NOT EXISTS "${DATA_PATH}")
  execute_process(COMMAND mkdir -p ${DATA_PATH})
endif()
function(download FILE_NAME FILE_HASH)
  message(STATUS "Checking and downloading ${FILE_NAME}")
  set(FILE_PATH ${DATA_PATH}/${FILE_NAME})
  set(STATUS_CODE 0)
  message(STATUS "start ${FILE_NAME}")
  if(EXISTS ${FILE_PATH})
    message(STATUS "found ${FILE_NAME}")
    file(MD5 ${FILE_PATH} EXISTING_FILE_HASH)
    if(${FILE_HASH} STREQUAL ${EXISTING_FILE_HASH})
      message(STATUS "same ${FILE_NAME}")
      message(STATUS "File already exists.")
    else()
      message(STATUS "diff ${FILE_NAME}")
      message(STATUS "File hash changes. Downloading now ...")
      file(DOWNLOAD ${DATA_DIR}/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    endif()
  else()
    message(STATUS "not found ${FILE_NAME}")
    message(STATUS "File doesn't exists. Downloading now ...")
    file(DOWNLOAD ${DATA_DIR}/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
    list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
    list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
  endif()
  if(${STATUS_CODE} EQUAL 0)
    message(STATUS "Download completed successfully!")
  else()
    message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
  endif()
endfunction()
download(tlr_yolox_s_batch_1.onnx ${PRETRAINED_MODEL_BATCH_1_HASH})
download(tlr_yolox_s_batch_4.onnx ${PRETRAINED_MODEL_BATCH_4_HASH})
download(tlr_yolox_s_batch_6.onnx ${PRETRAINED_MODEL_BATCH_6_HASH})
download(tlr_labels.txt ${LAMP_LABEL_HASH})

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )

  ament_auto_add_library(traffic_light_fine_detector_nodelet SHARED
    src/nodelet.cpp
  )

  target_include_directories(traffic_light_fine_detector_nodelet PUBLIC
    lib/include
  )

  target_link_libraries(traffic_light_fine_detector_nodelet
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
    ${OpenCV_LIB}
    stdc++fs
  )

  rclcpp_components_register_node(traffic_light_fine_detector_nodelet
    PLUGIN "traffic_light::TrafficLightFineDetectorNodelet"
    EXECUTABLE traffic_light_fine_detector_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    data
    launch
  )

else()
  message(STATUS "TrafficLightFineDetector won't be built, CUDA and/or TensorRT were not found.")
  # to avoid launch file missing without a gpu
  ament_auto_package(INSTALL_TO_SHARE
    launch
  )
endif()
