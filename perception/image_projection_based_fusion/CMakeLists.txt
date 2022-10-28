cmake_minimum_required(VERSION 3.14)
project(image_projection_based_fusion)
add_compile_options(-Wno-unknown-pragmas)

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
autoware_package()

# Build non-CUDA dependent nodes
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/fusion_node.cpp
  src/debugger.cpp
  src/utils/geometry.cpp
  src/utils/utils.cpp
  src/roi_cluster_fusion/node.cpp
  src/roi_detected_object_fusion/node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "image_projection_based_fusion::RoiDetectedObjectFusionNode"
  EXECUTABLE roi_detected_object_fusion_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "image_projection_based_fusion::RoiClusterFusionNode"
  EXECUTABLE roi_cluster_fusion_node
)

set(CUDA_VERBOSE OFF)

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

# Create folder to store trained models.
# NOTE: This must be created regardless of CUDA_AVAIL to be specified in ament_auto_package()
set(DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data)
execute_process(COMMAND mkdir -p ${DATA_PATH})

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  # Download trained models

  message(STATUS "start to download")
  function(download FILE_NAME FILE_HASH)
    message(STATUS "Checking and downloading ${FILE_NAME}")
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
        file(DOWNLOAD https://awf.ml.dev.web.auto/perception/models/pointpainting/v2/${FILE_NAME}
                      ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
        list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
        list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
      endif()
    else()
      message(STATUS "not found ${FILE_NAME}")
      message(STATUS "File doesn't exists. Downloading now ...")
      file(DOWNLOAD https://awf.ml.dev.web.auto/perception/models/pointpainting/v2/${FILE_NAME}
                    ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    endif()
    if(${STATUS_CODE} EQUAL 0)
      message(STATUS "Download completed successfully!")
    else()
      message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
    endif()
  endfunction()

  # default model
  download(pts_voxel_encoder_pointpainting.onnx 438dfecd962631ec8f011e0dfa2c6160)
  download(pts_backbone_neck_head_pointpainting.onnx e590a0b2bdcd35e01340cf4521bf149e)
  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)

  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )

  ament_auto_add_library(pointpainting_lib SHARED
    src/pointpainting_fusion/node.cpp
    src/pointpainting_fusion/pointpainting_trt.cpp
    src/pointpainting_fusion/voxel_generator.cpp
  )

  cuda_add_library(pointpainting_cuda_lib SHARED
    src/pointpainting_fusion/preprocess_kernel.cu
  )

  target_link_libraries(pointpainting_lib
    ${OpenCV_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${NVINFER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    pointpainting_cuda_lib
  )

  rclcpp_components_register_node(pointpainting_lib
    PLUGIN "image_projection_based_fusion::PointPaintingFusionNode"
    EXECUTABLE pointpainting_fusion_node
  )

else()
  message("Skipping build of some nodes due to missing dependencies")
endif()

ament_auto_package(INSTALL_TO_SHARE
    launch
    config
    data
)
