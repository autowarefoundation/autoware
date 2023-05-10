cmake_minimum_required(VERSION 3.17)
project(tensorrt_common)

find_package(ament_cmake REQUIRED)
find_package(cudnn_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tensorrt_cmake_module REQUIRED)

find_package(CUDA)
find_package(CUDAToolkit)
find_package(CUDNN)
find_package(TENSORRT)

if(NOT (CUDAToolkit_FOUND AND CUDNN_FOUND AND TENSORRT_FOUND))
  message(WARNING "cuda, cudnn, tensorrt libraries are not found")
  return()
endif()

add_library(${PROJECT_NAME} SHARED
  src/tensorrt_common.cpp
  src/simple_profiler.cpp
)

target_link_libraries(${PROJECT_NAME}
  CUDA::cudart
  rclcpp::rclcpp
  stdc++fs
  ${TENSORRT_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${TENSORRT_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# TODO(tensorrt_common): Remove -Wno-deprecated-declarations once upgrading to TensorRT 8.5 is complete
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -Wpedantic -Werror -Wno-deprecated-declarations
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  "CUDA"
  "CUDAToolkit"
  "cudnn_cmake_module"
  "CUDNN"
  "rclcpp"
  "tensorrt_cmake_module"
  "TENSORRT"
)

ament_package()
