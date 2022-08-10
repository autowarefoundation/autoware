cmake_minimum_required(VERSION 3.14)
project(lidar_apollo_instance_segmentation)

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
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVCAFFE_PARSER NAMES nvcaffe_parser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVPARSERS AND NVCAFFE_PARSER AND NVINFER_PLUGIN)
  if(CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVPARSERS: ${NVPARSERS}")
    message("NVCAFFE_PARSER: ${NVCAFFE_PARSER}")
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
    message("CUDNN is available!")
    message("CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  # download weight files
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
        file(DOWNLOAD https://awf.ml.dev.web.auto/perception/models/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
        list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
        list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
      endif()
    else()
      message(STATUS "not found ${FILE_NAME}")
      message(STATUS "File doesn't exists. Downloading now ...")
      file(DOWNLOAD https://awf.ml.dev.web.auto/perception/models/${FILE_NAME} ${FILE_PATH} STATUS DOWNLOAD_STATUS TIMEOUT 300)
      list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
      list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)
    endif()
    if(${STATUS_CODE} EQUAL 0)
      message(STATUS "Download completed successfully!")
    else()
      message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
    endif()
  endfunction()

  download(vlp-16.caffemodel f79f56a835893eb5289182dd06ce9905)
  download(hdl-64.caffemodel f79f56a835893eb5289182dd06ce9905)
  download(vls-128.caffemodel e5864f65c42d7d69a58fa7b01970d078)

  find_package(PCL REQUIRED)

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_MWAITXINTRIN_H_INCLUDED")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORCE_INLINES")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__STRICT_ANSI__")

  include_directories(
    lib/include
    ${CUDA_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )

  ament_auto_add_library(tensorrt_apollo_cnn_lib SHARED
    lib/src/TrtNet.cpp
  )

  target_link_libraries(tensorrt_apollo_cnn_lib
    ${NVINFER}
    ${NVCAFFE_PARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
    ${PCL_LIBRARIES}
  )

  target_compile_options(tensorrt_apollo_cnn_lib
    PUBLIC
      -Wno-deprecated-declarations
  )

  ament_auto_add_library(lidar_apollo_instance_segmentation SHARED
    src/node.cpp
    src/detector.cpp
    src/log_table.cpp
    src/feature_generator.cpp
    src/feature_map.cpp
    src/cluster2d.cpp
    src/debugger.cpp
  )

  target_link_libraries(lidar_apollo_instance_segmentation
    tensorrt_apollo_cnn_lib
  )

  rclcpp_components_register_node(lidar_apollo_instance_segmentation
    PLUGIN "LidarInstanceSegmentationNode"
    EXECUTABLE lidar_apollo_instance_segmentation_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    launch
    config
    data
  )

else()
  # to avoid launch file missing without a gpu
  ament_auto_package(INSTALL_TO_SHARE
    launch
  )

endif()
