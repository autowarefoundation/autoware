cmake_minimum_required(VERSION 3.5)
project(tensorrt_yolo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations -Werror)
endif()

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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

# Download onnx
find_program(GDOWN_AVAIL "gdown")
if(NOT GDOWN_AVAIL)
  message("gdown: command not found. External files could not be downloaded.")
endif()
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if(NOT EXISTS "${PATH}")
  execute_process(COMMAND mkdir -p ${PATH})
endif()
# yolov3
set(FILE "${PATH}/yolov3.onnx")
message(STATUS "Checking and downloading yolov3.onnx")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND gdown "https://drive.google.com/uc?id=1ZYoBqVynmO5kKntyN56GEbELRpuXG8Ym" -O ${PATH}/yolov3.onnx
    ERROR_QUIET
  )
endif()

# yolov4
set(FILE "${PATH}/yolov4.onnx")
message(STATUS "Checking and downloading yolov4.onnx")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND gdown "https://drive.google.com//uc?id=1vkNmSwcIpTkJ_-BrKhxtit0PBJeJYTVX" -O ${PATH}/yolov4.onnx
    ERROR_QUIET
  )
endif()

# yolov4-tiny
set(FILE "${PATH}/yolov4-tiny.onnx")
message(STATUS "Checking and downloading yolov4.onnx")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND gdown "https://drive.google.com/uc?id=1rUHjV_JfkmlVFgb20XXrOMWo_HZAjrFh" -O ${PATH}/yolov4-tiny.onnx
    ERROR_QUIET
  )
endif()

# yolov5s
set(FILE "${PATH}/yolov5s.onnx")
message(STATUS "Checking and downloading yolov5s.onnx")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND gdown "https://drive.google.com//uc?id=1CF21nQWigwCPTr5psQZXg6cBQIOYKbad" -O ${PATH}/yolov5s.onnx
    ERROR_QUIET
  )
endif()

# yolov5m
set(FILE "${PATH}/yolov5m.onnx")
message(STATUS "Checking and downloading yolov5m.onnx")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND gdown "https://drive.google.com//uc?id=1a1h50KJH6slwmjKZpPlS-errukF-BrgG" -O ${PATH}/yolov5m.onnx
    ERROR_QUIET
  )
endif()

# yolov5l
set(FILE "${PATH}/yolov5l.onnx")
message(STATUS "Checking and downloading yolov5l.onnx")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND gdown "https://drive.google.com/uc?id=1xO8S92Cq7qrmx93UHHyA7Cd7-dJsBDP8" -O ${PATH}/yolov5l.onnx
    ERROR_QUIET
  )
endif()

# yolov5x
set(FILE "${PATH}/yolov5x.onnx")
message(STATUS "Checking and downloading yolov5x.onnx")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND gdown "https://drive.google.com/uc?id=1kAHuNJUCxpD-yWrS6t95H3zbAPfClLxI" -O ${PATH}/yolov5x.onnx
    ERROR_QUIET
  )
endif()

set(FILE "${PATH}/coco.names")
message(STATUS "Checking and downloading coco.names")
if(NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(
    COMMAND gdown "https://drive.google.com/uc?id=19wXD23PE16kJDkZ7j2W3Ijvx5I1kO9kJ" -O ${PATH}/coco.names
    ERROR_QUIET
  )
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
    ${CUDA_curand_LIBRARY}
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

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_auto_package(INSTALL_TO_SHARE
    config
    data
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
    data
    launch
  )
endif()
