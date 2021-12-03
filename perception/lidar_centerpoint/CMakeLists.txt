cmake_minimum_required(VERSION 3.5)
project(lidar_centerpoint)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

option(TORCH_AVAIL "Torch available" OFF)
if(CUDA_FOUND)
  set(Torch_DIR /usr/local/libtorch/share/cmake/Torch)
  find_package(Torch)
  if(TORCH_FOUND)
    if(CUDA_VERBOSE)
      message(STATUS "TORCH_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")
      message(STATUS "TORCH_LIBRARIES: ${TORCH_LIBRARIES}")
    endif()
    set(TORCH_AVAIL ON)
  else()
  message("Torch NOT FOUND")
  set(TORCH_AVAIL OFF)
  endif()
endif()

if(TRT_AVAIL AND CUDA_AVAIL AND TORCH_AVAIL)
# Download trained models
  find_program(GDOWN_AVAIL "gdown")
  if(NOT GDOWN_AVAIL)
    message("gdown: command not found. External files could not be downloaded.")
  endif()

  set(DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data)
  execute_process(COMMAND mkdir -p ${DATA_PATH})

  function(download FILE_NAME GFILE_ID FILE_HASH)
    # https://drive.google.com/file/d/GFILE_ID/view
    message(STATUS "Checking and downloading ${FILE_NAME}")
    set(FILE_PATH ${DATA_PATH}/${FILE_NAME})
    if(EXISTS ${FILE_PATH})
      file(MD5 ${FILE_PATH} EXISTING_FILE_HASH)
      if(NOT ${FILE_HASH} EQUAL ${EXISTING_FILE_HASH})
        message(STATUS "... file hash changes. Downloading now ...")
        execute_process(COMMAND gdown --quiet https://drive.google.com//uc?id=${GFILE_ID} -O ${FILE_PATH})
      endif()
    else()
      message(STATUS "... file doesn't exists. Downloading now ...")
      execute_process(COMMAND gdown --quiet https://drive.google.com//uc?id=${GFILE_ID} -O ${FILE_PATH})
    endif()
  endfunction()

  # default model
  download(pts_voxel_encoder_default.onnx 1_8OCQmrPm_R4ZVh70QsS9HZo6uGrlbgz 01b860612e497591c4375d90dff61ef7)
  download(pts_voxel_encoder_default.pt 1RZ7cuDnI-RBrDiWe-2vEs16mR_z0e9Uo 33136caa97e3bcef2cf3e04bbc93d1e4)
  download(pts_backbone_neck_head_default.onnx 1UxDyt8T-TMJS7Ujx-1vbbqGRfDbMUZg2 e23a8ad4ea440f923e44dbe072b070da)
  download(pts_backbone_neck_head_default.pt 1toAhmOriX8bwVI-ohuas9_2EBZnltoXh eb0df29b30acf9c1082ac4490af0bbc5)

  # aip_x2 model
  download(pts_voxel_encoder_aip_x2.onnx 1x-NAHQ3W0lbLmjJlrL6Nhvdq8yz6Ux0n 65eeb95c5e48ebfe6894146cdb48c160)
  download(pts_voxel_encoder_aip_x2.pt 1jzKopAhXWjnEgo_v8rtYy0hQIayUE-oL 4db81ce8edc6571aa0afb1ae43ee72e9)
  download(pts_backbone_neck_head_aip_x2.onnx 1l2fdIQcBWr3-6stVoNkudnL4OZaPqmNT a33c8910fd9c9c910b10904d3cd96717)
  download(pts_backbone_neck_head_aip_x2.pt 18iOAlRsjvcWoUG9KiL1PlD7OY5mi9BSw 274fdf1580dd899e36c050c1366f1883)

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(
    lib/include
    ${CUDA_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
  )

  ### centerpoint ###
  ament_auto_add_library(centerpoint SHARED
    lib/src/pointcloud_densification.cpp
    lib/src/voxel_generator.cpp
    lib/src/centerpoint_trt.cpp
    lib/src/tensorrt_wrapper.cpp
    lib/src/network_trt.cpp
  )

  target_link_libraries(centerpoint
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${TORCH_LIBRARIES}
  )

  ## node ##
  ament_auto_add_library(lidar_centerpoint_component SHARED
    src/node.cpp
  )

  target_link_libraries(lidar_centerpoint_component
    centerpoint
  )

  rclcpp_components_register_node(lidar_centerpoint_component
    PLUGIN "centerpoint::LidarCenterPointNode"
    EXECUTABLE lidar_centerpoint_node
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
      data
      config
  )
else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
  ament_auto_package(
    INSTALL_TO_SHARE
      launch
  )
endif()
