cmake_minimum_required(VERSION 3.14)
project(autoware_image_projection_based_fusion)
# add_compile_options(-Wno-unknown-pragmas)
add_compile_options(-Wno-unknown-pragmas -fopenmp)

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
autoware_package()

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Build non-CUDA dependent nodes
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/fusion_node.cpp
  src/debugger.cpp
  src/utils/geometry.cpp
  src/utils/utils.cpp
  src/roi_cluster_fusion/node.cpp
  src/roi_detected_object_fusion/node.cpp
  src/segmentation_pointcloud_fusion/node.cpp
  src/roi_pointcloud_fusion/node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::image_projection_based_fusion::RoiDetectedObjectFusionNode"
  EXECUTABLE roi_detected_object_fusion_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::image_projection_based_fusion::RoiClusterFusionNode"
  EXECUTABLE roi_cluster_fusion_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::image_projection_based_fusion::SegmentPointCloudFusionNode"
  EXECUTABLE segmentation_pointcloud_fusion_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::image_projection_based_fusion::RoiPointCloudFusionNode"
  EXECUTABLE roi_pointcloud_fusion_node
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

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)

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
    src/pointpainting_fusion/pointcloud_densification.cpp
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
    PLUGIN "autoware::image_projection_based_fusion::PointPaintingFusionNode"
    EXECUTABLE pointpainting_fusion_node
  )

  install(
    TARGETS pointpainting_cuda_lib
    DESTINATION lib
  )
else()
  message("Skipping build of some nodes due to missing dependencies")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_calc_iou_functions
    test/test_calc_iou_functions.cpp
  )
  ament_auto_add_gtest(test_utils
    test/test_utils.cpp
  )
  ament_auto_add_gtest(test_geometry
    test/test_geometry.cpp
  )
  # test needed cuda, tensorRT and cudnn
  if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
    ament_auto_add_gtest(test_pointpainting
      test/test_pointpainting_fusion.cpp
    )
  endif()

endif()

ament_auto_package(INSTALL_TO_SHARE
    launch
    config
)
