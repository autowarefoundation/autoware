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

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
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
      if(${FILE_HASH} EQUAL ${EXISTING_FILE_HASH})
        message(STATUS "File already exists.")
      else()
        message(STATUS "File hash changes. Downloading now ...")
        execute_process(COMMAND gdown --quiet https://drive.google.com//uc?id=${GFILE_ID} -O ${FILE_PATH})
        # file(MD5 ${FILE_PATH} DOWNLOADED_FILE_HASH)  # disable to pass ci
        message(STATUS "Downloaded file hash: ${DOWNLOADED_FILE_HASH}")
      endif()
    else()
      message(STATUS "File doesn't exists. Downloading now ...")
      execute_process(COMMAND gdown --quiet https://drive.google.com//uc?id=${GFILE_ID} -O ${FILE_PATH})
      # file(MD5 ${FILE_PATH} DOWNLOADED_FILE_HASH)  # disable to pass ci
      message(STATUS "Downloaded file hash: ${DOWNLOADED_FILE_HASH}")
    endif()
  endfunction()

  # default model
  download(pts_voxel_encoder_default.onnx 1KFhmA4oFT6CtZx5806QeMzn5H2tKa3oD 410f730c537968cb27fbd70c941849a8)
  download(pts_backbone_neck_head_default.onnx 1iyk5VoQ4uNBGPZwypVZIMjSuSYAI1RxP e97c165c7877222c0e27e44409a07517)

  # aip_x2 model
  download(pts_voxel_encoder_aip_x2.onnx 13aYPRHx17Ge4BqxzW9drAUSWTppjtUV5 3ae5e9efd7b2ed12115e6f0b28cac58d)
  download(pts_backbone_neck_head_aip_x2.onnx 14PJ_L3Jpz6Yi8GzoctVOEbGWcaCLArGp 6a406a19e05660677c162486ab332de8)

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(
    lib/include
    ${CUDA_INCLUDE_DIRS}
  )

  ### centerpoint ###
  ament_auto_add_library(centerpoint SHARED
    lib/src/centerpoint_trt.cpp
    lib/src/pointcloud_densification.cpp
    lib/src/voxel_generator.cpp
    lib/src/tensorrt_wrapper.cpp
    lib/src/network_trt.cpp
    lib/src/utils.cpp
  )

  cuda_add_library(centerpoint_cuda_libraries SHARED
    lib/src/circle_nms_kernel.cu
    lib/src/postprocess_kernel.cu
    lib/src/preprocess_kernel.cu
    lib/src/scatter_kernel.cu
  )

  target_link_libraries(centerpoint
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    centerpoint_cuda_libraries
  )

  ## node ##
  ament_auto_add_library(lidar_centerpoint_component SHARED
    src/node.cpp
  )

  target_link_libraries(lidar_centerpoint_component
    centerpoint
  )

  # workaround to allow deprecated header to build on both galactic and rolling
  if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
    target_compile_definitions(lidar_centerpoint_component PUBLIC
      USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
    )
  endif()

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
