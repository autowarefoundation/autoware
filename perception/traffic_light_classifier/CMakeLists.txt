cmake_minimum_required(VERSION 3.14)
project(traffic_light_classifier)

find_package(autoware_cmake REQUIRED)
autoware_package()

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)
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

# cspell: ignore EFFICIENTNET, MOBILENET
# Download caffemodel and prototxt
set(EFFICIENTNET_BATCH_1_HASH 82baba4fcf1abe0c040cd55005e34510)
set(EFFICIENTNET_BATCH_4_HASH 21b549c2fe4fbb20d32cc019e6d70cd7)
set(EFFICIENTNET_BATCH_6_HASH 28b408710bcb24f4cdd4d746301d4e78)
set(MOBILENET_BATCH_1_HASH caa51f2080aa2df943e4f884c41898eb)
set(MOBILENET_BATCH_4_HASH c2beaf60210f471debfe72b86d076ca0)
set(MOBILENET_BATCH_6_HASH 47255a11bde479320d703f1f45db1242)
set(LAMP_LABEL_HASH e9f45efb02f2a9aa8ac27b3d5c164905)
set(AWS_DIR https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2)
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
      file(DOWNLOAD ${AWS_DIR}/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    endif()
  else()
    message(STATUS "not found ${FILE_NAME}")
    message(STATUS "File doesn't exists. Downloading now ...")
    file(DOWNLOAD ${AWS_DIR}/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
    list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
    list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
  endif()
  if(${STATUS_CODE} EQUAL 0)
    message(STATUS "Download completed successfully!")
  else()
    message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
  endif()
endfunction()
download(traffic_light_classifier_mobilenetv2_batch_1.onnx ${MOBILENET_BATCH_1_HASH})
download(traffic_light_classifier_mobilenetv2_batch_4.onnx ${MOBILENET_BATCH_4_HASH})
download(traffic_light_classifier_mobilenetv2_batch_6.onnx ${MOBILENET_BATCH_6_HASH})
download(traffic_light_classifier_efficientNet_b1_batch_1.onnx ${EFFICIENTNET_BATCH_1_HASH})
download(traffic_light_classifier_efficientNet_b1_batch_4.onnx ${EFFICIENTNET_BATCH_4_HASH})
download(traffic_light_classifier_efficientNet_b1_batch_6.onnx ${EFFICIENTNET_BATCH_6_HASH})
download(lamp_labels.txt 4b2cf910d97d05d464e7c26901af3d4c)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  add_definitions(-DENABLE_GPU)

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )

  ament_auto_add_library(traffic_light_classifier_nodelet SHARED
    src/color_classifier.cpp
    src/cnn_classifier.cpp
    src/nodelet.cpp
  )
  target_link_libraries(traffic_light_classifier_nodelet
    ${OpenCV_LIBRARIES}
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
    stdc++fs
  )
  rclcpp_components_register_node(traffic_light_classifier_nodelet
    PLUGIN "traffic_light::TrafficLightClassifierNodelet"
    EXECUTABLE traffic_light_classifier_node
  )

  ament_auto_add_library(single_image_debug_inference_node SHARED
    src/cnn_classifier.cpp
    src/color_classifier.cpp
    src/nodelet.cpp
    src/single_image_debug_inference_node.cpp
  )
  target_link_libraries(single_image_debug_inference_node
    ${OpenCV_LIBRARIES}
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
    stdc++fs
    opencv_core
    opencv_highgui
  )
  rclcpp_components_register_node(single_image_debug_inference_node
    PLUGIN "traffic_light::SingleImageDebugInferenceNode"
    EXECUTABLE single_image_debug_inference
  )

  ament_auto_package(INSTALL_TO_SHARE
    data
    launch
  )

else()
  message(STATUS "CUDA and/or TensorRT were not found. build only color classifier")

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
  )

  ament_auto_add_library(traffic_light_classifier_nodelet SHARED
    src/color_classifier.cpp
    src/nodelet.cpp
  )
  target_link_libraries(traffic_light_classifier_nodelet
    ${OpenCV_LIBRARIES}
  )

  rclcpp_components_register_node(traffic_light_classifier_nodelet
    PLUGIN "traffic_light::TrafficLightClassifierNodelet"
    EXECUTABLE traffic_light_classifier_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    data
    launch
  )

endif()
