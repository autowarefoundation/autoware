cmake_minimum_required(VERSION 3.5)
project(lidar_apollo_instance_segmentation)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if (CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if (CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif ()
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif (CUDA_FOUND)

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVCAFFE_PARSER NAMES nvcaffe_parser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVPARSERS AND NVCAFFE_PARSER AND NVINFER_PLUGIN)
  if (CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVPARSERS: ${NVPARSERS}")
    message("NVCAFFE_PARSER: ${NVCAFFE_PARSER}")
  endif ()
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
DOC "CUDNN library." )
if(CUDNN_LIBRARY)
  if (CUDA_VERBOSE)
    message("CUDNN is available!")
    message("CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif ()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

find_program(GDOWN_AVAIL "gdown")
if (NOT GDOWN_AVAIL)
  message("gdown: command not found. External files could not be downloaded.")
endif()
if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  # download weight files
  set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
  if (NOT EXISTS "${PATH}")
    execute_process(COMMAND mkdir -p ${PATH})
  endif()
  set(FILE "${PATH}/vlp-16.caffemodel")
  message(STATUS "Checking and downloading vlp-16.caffemodel")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND gdown "https://drive.google.com/uc?id=1bbO_eeGG5HUqyUiAYjOd6hUn-Zma0mMJ" -O ${PATH}/vlp-16.caffemodel)
  endif()
  set(FILE "${PATH}/hdl-64.caffemodel")
  message(STATUS "Checking and downloading hdl-64.caffemodel")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND gdown "https://drive.google.com/uc?id=1ZdB6V3jua3GmtYuY9NR1nc9QZe_ChjkP" -O ${PATH}/hdl-64.caffemodel)
  endif()
  set(FILE "${PATH}/vls-128.caffemodel")
  message(STATUS "Checking and downloading vls-128.caffemodel")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND gdown "https://drive.google.com/uc?id=1izpNotNxS6mrYIzJwHQ_EyX_IPDU-MBr" -O ${PATH}/vls-128.caffemodel)
  endif()

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
  find_package(PCL REQUIRED)

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations -Werror)
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
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
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    ${PCL_LIBRARIES}
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

  ament_auto_package(INSTALL_TO_SHARE
    launch
    config
    data
  )

else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

  # to avoid launch file missing without a gpu
  ament_auto_package(INSTALL_TO_SHARE
    launch
  )

endif()
