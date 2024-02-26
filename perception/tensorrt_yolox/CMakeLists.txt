cmake_minimum_required(VERSION 3.17)
project(tensorrt_yolox)

find_package(tensorrt_common)
if(NOT ${tensorrt_common_FOUND})
  message(WARNING "The tensorrt_common package is not found. Please check its dependencies.")
  return()
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
else()
  message(WARNING "CUDA is not found. preprocess acceleration using CUDA will not be available.")
endif()
find_package(OpenMP)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

##########
# tensorrt_yolox
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tensorrt_yolox.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
)

if(CMAKE_CUDA_COMPILER)
  # Officially, add_library supports .cu file compilation.
  # However, as of cmake 3.22.1, it seems to fail compilation because compiler flags for
  # C++ are directly passed to nvcc (they are originally space separated
  # but nvcc assume comma separated as argument of `-Xcompiler` option).
  # That is why `cuda_add_library` is used here.
  cuda_add_library(${PROJECT_NAME}_gpu_preprocess
    SHARED
    src/preprocess.cu
  )

  target_include_directories(${PROJECT_NAME}_gpu_preprocess PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  target_link_libraries(${PROJECT_NAME}
    ${tensorrt_common_LIBRARIES}
    ${PROJECT_NAME}_gpu_preprocess
  )
else()
  target_link_libraries(${PROJECT_NAME}
    ${tensorrt_common_LIBRARIES}
  )
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

ament_auto_add_library(yolox_single_image_inference_node SHARED
  src/yolox_single_image_inference_node.cpp
)

ament_target_dependencies(yolox_single_image_inference_node
  OpenCV
)

target_link_libraries(yolox_single_image_inference_node
  ${PROJECT_NAME}
  stdc++fs
)

target_compile_definitions(yolox_single_image_inference_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(yolox_single_image_inference_node
  PLUGIN "tensorrt_yolox::YoloXSingleImageInferenceNode"
  EXECUTABLE yolox_single_image_inference
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/tensorrt_yolox_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  OpenCV
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

target_compile_definitions(${PROJECT_NAME}_node PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "tensorrt_yolox::TrtYoloXNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
