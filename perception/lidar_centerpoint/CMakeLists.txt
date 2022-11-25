cmake_minimum_required(VERSION 3.14)
project(lidar_centerpoint)

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
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  # Note: cublas_device was depreciated in CUDA version 9.2
  #       https://forums.developer.nvidia.com/t/where-can-i-find-libcublas-device-so-or-libcublas-device-a/67251/4
  #       In LibTorch, CUDA_cublas_device_LIBRARY is used.
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVONNXPARSER: ${NVONNXPARSER}")
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

message(STATUS "start to download")
if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
# Download trained models
  set(DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data)
  set(DOWNLOAD_BASE_URL https://awf.ml.dev.web.auto/perception/models/centerpoint)
  execute_process(COMMAND mkdir -p ${DATA_PATH})

  function(download VERSION FILE_NAME FILE_HASH)
    message(STATUS "Checking and downloading ${FILE_NAME} ")
    set(DOWNLOAD_URL ${DOWNLOAD_BASE_URL}/${VERSION}/${FILE_NAME})
    set(FILE_PATH ${DATA_PATH}/${FILE_NAME})
    set(STATUS_CODE 0)
    message(STATUS "start ${FILE_NAME}")
    if(EXISTS ${FILE_PATH})
      message(STATUS "found ${FILE_NAME}")
      file(MD5 ${FILE_PATH} EXISTING_FILE_HASH)
      if("${FILE_HASH}" STREQUAL "${EXISTING_FILE_HASH}")
        message(STATUS "same ${FILE_NAME}")
        message(STATUS "File already exists.")
      else()
        message(STATUS "diff ${FILE_NAME}")
        message(STATUS "File hash changes. Downloading now ...")
        file(DOWNLOAD ${DOWNLOAD_URL} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
        list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
        list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
      endif()
    else()
      message(STATUS "not found ${FILE_NAME}")
      message(STATUS "File doesn't exists. Downloading now ...")
      file(DOWNLOAD ${DOWNLOAD_URL} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    endif()
    if(${STATUS_CODE} EQUAL 0)
      message(STATUS "Download completed successfully!")
    else()
      message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE} ${DOWNLOAD_URL}")
    endif()
  endfunction()

  # centerpoint
  download(v1 pts_voxel_encoder_centerpoint.onnx 94dcf01ccd53ef14a79787afaada63b4)
  download(v1 pts_backbone_neck_head_centerpoint.onnx 7046995e502e11f35c9459668d8afde3)

  # centerpoint_tiny
  download(v1 pts_voxel_encoder_centerpoint_tiny.onnx 410f730c537968cb27fbd70c941849a8)
  download(v1 pts_backbone_neck_head_centerpoint_tiny.onnx e97c165c7877222c0e27e44409a07517)

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(
    include
    ${CUDA_INCLUDE_DIRS}
  )

  ### centerpoint ###
  ament_auto_add_library(centerpoint_lib SHARED
    lib/centerpoint_trt.cpp
    lib/detection_class_remapper.cpp
    lib/utils.cpp
    lib/ros_utils.cpp
    lib/network/network_trt.cpp
    lib/network/tensorrt_wrapper.cpp
    lib/postprocess/non_maximum_suppression.cpp
    lib/preprocess/pointcloud_densification.cpp
    lib/preprocess/voxel_generator.cpp
  )

  cuda_add_library(centerpoint_cuda_lib SHARED
    lib/postprocess/circle_nms_kernel.cu
    lib/postprocess/postprocess_kernel.cu
    lib/network/scatter_kernel.cu
    lib/preprocess/preprocess_kernel.cu
  )

  target_link_libraries(centerpoint_lib
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    centerpoint_cuda_lib
  )

  target_include_directories(centerpoint_lib
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  # To suppress unknown-pragmas error. The root-cause is CUB library in CUDA 11.6.
  # This issue was fixed by https://github.com/NVIDIA/cub/commit/7d608bf1dc14553e2fb219eabeed80b76621b6fe
  target_include_directories(centerpoint_lib
    SYSTEM PUBLIC
      ${CUDA_INCLUDE_DIRS}
  )

  ## node ##
  ament_auto_add_library(lidar_centerpoint_component SHARED
    src/node.cpp
  )

  target_link_libraries(lidar_centerpoint_component
    centerpoint_lib
  )

  rclcpp_components_register_node(lidar_centerpoint_component
    PLUGIN "centerpoint::LidarCenterPointNode"
    EXECUTABLE lidar_centerpoint_node
  )

  ## single inference node ##
  ament_auto_add_library(single_inference_lidar_centerpoint_component SHARED
    src/single_inference_node.cpp
  )

  target_link_libraries(single_inference_lidar_centerpoint_component
    centerpoint_lib
  )

  rclcpp_components_register_node(single_inference_lidar_centerpoint_component
    PLUGIN "centerpoint::SingleInferenceLidarCenterPointNode"
    EXECUTABLE single_inference_lidar_centerpoint_node
  )

  install(PROGRAMS
    scripts/lidar_centerpoint_visualizer.py
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_export_dependencies(ament_cmake_python)

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
      data
      config
  )

  install(
    TARGETS centerpoint_cuda_lib
    DESTINATION lib
  )
else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
  )
endif()
