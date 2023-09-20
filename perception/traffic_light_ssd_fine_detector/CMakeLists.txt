cmake_minimum_required(VERSION 3.14)
project(traffic_light_ssd_fine_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

# TODO(traffic_light_ssd_fine_detector): Remove once upgrading to TensorRT 8.5 is complete
add_compile_options(-Wno-deprecated-declarations)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

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

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )

  ### ssd ###
  cuda_add_library(gather_topk_plugin SHARED
    lib/src/plugins/gather_topk_kernel.cu
    lib/src/plugins/gather_topk.cpp
  )

  cuda_add_library(grid_priors_plugin SHARED
    lib/src/plugins/grid_priors_kernel.cu
    lib/src/plugins/grid_priors.cpp
  )

  ament_auto_add_library(ssd SHARED
    lib/src/trt_ssd.cpp
  )

  target_include_directories(ssd PUBLIC
    lib/include
  )

  target_include_directories(gather_topk_plugin PUBLIC
    lib/include
  )

  target_include_directories(grid_priors_plugin PUBLIC
    lib/include
  )

  target_link_libraries(ssd
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDNN_LIBRARY}
    gather_topk_plugin
    grid_priors_plugin
  )

  ament_auto_add_library(traffic_light_ssd_fine_detector_nodelet SHARED
    src/nodelet.cpp
  )

  target_link_libraries(traffic_light_ssd_fine_detector_nodelet
    ${OpenCV_LIB}
    stdc++fs
    ssd
    gather_topk_plugin
    grid_priors_plugin
  )

  rclcpp_components_register_node(traffic_light_ssd_fine_detector_nodelet
    PLUGIN "traffic_light::TrafficLightSSDFineDetectorNodelet"
    EXECUTABLE traffic_light_ssd_fine_detector_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    launch
  )

  install(
    TARGETS
      gather_topk_plugin
      grid_priors_plugin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

else()
  message(STATUS "TrafficLightSSDFineDetector won't be built, CUDA and/or TensorRT were not found.")
  # to avoid launch file missing without a gpu
  ament_auto_package(INSTALL_TO_SHARE
    launch
  )
endif()
