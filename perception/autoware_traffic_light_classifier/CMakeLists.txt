cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_classifier)

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
    src/classifier/color_classifier.cpp
    src/classifier/cnn_classifier.cpp
    src/traffic_light_classifier_node.cpp
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
    PLUGIN "autoware::traffic_light::TrafficLightClassifierNodelet"
    EXECUTABLE traffic_light_classifier_node
  )

  ament_auto_add_library(single_image_debug_inference_node SHARED
    src/classifier/cnn_classifier.cpp
    src/classifier/color_classifier.cpp
    src/traffic_light_classifier_node.cpp
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
    PLUGIN "autoware::traffic_light::SingleImageDebugInferenceNode"
    EXECUTABLE single_image_debug_inference
  )

  ament_auto_package(INSTALL_TO_SHARE
    launch
    config
  )

else()
  message(STATUS "CUDA and/or TensorRT were not found. build only color classifier")

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
  )

  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/classifier/color_classifier.cpp
    src/traffic_light_classifier_node.cpp
  )
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
  )

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::traffic_light::TrafficLightClassifierNodelet"
    EXECUTABLE traffic_light_classifier_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    launch
    config
  )

endif()
