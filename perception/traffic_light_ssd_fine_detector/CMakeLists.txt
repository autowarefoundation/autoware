cmake_minimum_required(VERSION 3.5)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

project(traffic_light_ssd_fine_detector)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if (CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if (CUDA_VERBOSE)
    message(STATUS "CUDA is available!")
    message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif ()
  set(CUDA_AVAIL ON)
else()
  message(STATUS "CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif (CUDA_FOUND)

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVONNXPARSER nvonnxparser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  if (CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVPARSERS: ${NVPARSERS}")
    message(STATUS "NVINFER_PLUGIN: ${NVINFER_PLUGIN}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif ()
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
DOC "CUDNN library." )
if(CUDNN_LIBRARY)
  if (CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif ()
  set(CUDNN_AVAIL ON)
else()
  message(STATUS "CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# Download caffemodel and prototxt
set(PRETRAINED_MODEL_LINK "https://drive.google.com/uc?id=1USFDPRH9JrVdGoqt27qHjRgittwc0kcO")
set(PRETRAINED_MODEL_HASH 34ce7f2cbacbf6da8bc35769f027b73f)
set(LAMP_LABEL_LINK "https://drive.google.com/uc?id=1hPcKvKgKz0fqEo0cNAXH7roEletqZErL")
set(LAMP_LABEL_HASH e9f45efb02f2a9aa8ac27b3d5c164905)

find_program(GDOWN_AVAIL "gdown")
if (NOT GDOWN_AVAIL)
  message(STATUS "gdown: command not found. External files could not be downloaded.")
endif()
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if (NOT EXISTS "${PATH}")
  execute_process(COMMAND mkdir -p ${PATH})
endif()

set(FILE "${PATH}/mb2-ssd-lite-tlr.onnx")
message(STATUS "Checking and downloading mb2-ssd-lite-tlr.onnx")
if (EXISTS "${FILE}")
  file(MD5 "${FILE}" EXISTING_FILE_HASH)
  if (NOT "${PRETRAINED_MODEL_HASH}" EQUAL "${EXISTING_FILE_HASH}")
    message(STATUS "... file hash changed. Downloading now ...")
    execute_process(COMMAND gdown --quiet "${PRETRAINED_MODEL_LINK}" -O ${PATH}/mb2-ssd-lite-tlr.onnx)
  endif()
else()
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND gdown --quiet "${PRETRAINED_MODEL_LINK}" -O ${PATH}/mb2-ssd-lite-tlr.onnx)
endif()

set(FILE "${PATH}/voc_labels_tl.txt")
message(STATUS "Checking and downloading voc_labels_tl.txt")
if (EXISTS "${FILE}")
  file(MD5 "${FILE}" EXISTING_FILE_HASH)
  if (NOT "${LAMP_LABEL_HASH}" EQUAL "${EXISTING_FILE_HASH}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND gdown --quiet "${LAMP_LABEL_LINK}" -O ${PATH}/voc_labels_tl.txt)
  endif()
else()
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND gdown --quiet "${LAMP_LABEL_LINK}" -O ${PATH}/voc_labels_tl.txt)
endif()

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )

  set(CMAKE_CXX_FLAGS "-O2 -Wall -Wunused-variable ${CMAKE_CXX_FLAGS} -fpic -std=c++11 -pthread")

  ### ssd ###
  ament_auto_add_library(ssd SHARED
    lib/src/trt_ssd.cpp
  )

  target_include_directories(ssd PUBLIC
    lib/include
  )

  target_link_libraries(ssd
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
  )

  ament_auto_add_library(traffic_light_ssd_fine_detector_nodelet SHARED
    src/nodelet.cpp
  )

  target_link_libraries(traffic_light_ssd_fine_detector_nodelet
    ${OpenCV_LIB}
    ssd
  )

  rclcpp_components_register_node(traffic_light_ssd_fine_detector_nodelet
    PLUGIN "traffic_light::TrafficLightSSDFineDetectorNodelet"
    EXECUTABLE traffic_light_ssd_fine_detector_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    data
    launch
  )

else()
  message(STATUS "TrafficLightSSDFineDetector won't be built, CUDA and/or TensorRT were not found.")
endif()
