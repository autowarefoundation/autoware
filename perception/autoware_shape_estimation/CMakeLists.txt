cmake_minimum_required(VERSION 3.14)
project(autoware_shape_estimation)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(CUDA)
find_package(CUDNN)
find_package(TENSORRT)

set(SHAPE_ESTIMATION_DEPENDENCIES
  PCL
  OpenCV
  Eigen3
)

set(${PROJECT_NAME}_SOURCES
  lib/shape_estimator.cpp
  lib/model/bounding_box.cpp
  lib/model/convex_hull.cpp
  lib/model/cylinder.cpp
  lib/filter/car_filter.cpp
  lib/filter/bus_filter.cpp
  lib/filter/truck_filter.cpp
  lib/filter/trailer_filter.cpp
  lib/filter/no_filter.cpp
  lib/filter/utils.cpp
  lib/corrector/utils.cpp
  lib/corrector/no_corrector.cpp
  lib/corrector/vehicle_corrector.cpp
  lib/corrector/reference_shape_size_corrector.cpp
)

if(${CUDA_FOUND} AND ${CUDNN_FOUND} AND ${TENSORRT_FOUND})
  message("CUDA found, including CUDA-specific sources")
  list(APPEND ${PROJECT_NAME}_SOURCES lib/tensorrt_shape_estimator.cpp)
  add_definitions(-DUSE_CUDA)
else()
  message("CUDA not found, excluding CUDA-specific sources")
endif()

ament_auto_add_library(${PROJECT_NAME}_lib
  SHARED
  ${${PROJECT_NAME}_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}_lib ${SHAPE_ESTIMATION_DEPENDENCIES})

target_include_directories(${PROJECT_NAME}_lib
  SYSTEM PUBLIC
  "${PCL_INCLUDE_DIRS}"
  "${EIGEN3_INCLUDE_DIR}"
)

if(${CUDA_FOUND} AND ${CUDNN_FOUND} AND ${TENSORRT_FOUND})
target_include_directories(${PROJECT_NAME}_lib
  SYSTEM PUBLIC
  "${TENSORRT_INCLUDE_DIRS}"
)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/shape_estimation_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(${PROJECT_NAME} ${SHAPE_ESTIMATION_DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

if(${CUDA_FOUND} AND ${CUDNN_FOUND} AND ${TENSORRT_FOUND})
  target_link_libraries(${PROJECT_NAME}
    ${TENSORRT_LIBRARIES}
  )
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::shape_estimation::ShapeEstimationNode"
  EXECUTABLE shape_estimation_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

## Tests
if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE test_files test/**/*.cpp)
  ament_add_ros_isolated_gtest(test_shape_estimation ${test_files})

  target_link_libraries(test_shape_estimation
    ${PROJECT_NAME}
  )
endif()
