cmake_minimum_required(VERSION 3.14)
project(shape_estimation)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(SHAPE_ESTIMATION_DEPENDENCIES
  PCL
  OpenCV
  Eigen3
)

ament_auto_add_library(shape_estimation_lib SHARED
  lib/shape_estimator.cpp
  lib/model/bounding_box.cpp
  lib/model/convex_hull.cpp
  lib/model/cylinder.cpp
  lib/filter/car_filter.cpp
  lib/filter/bus_filter.cpp
  lib/filter/truck_filter.cpp
  lib/filter/no_filter.cpp
  lib/filter/utils.cpp
  lib/corrector/no_corrector.cpp
  lib/corrector/utils.cpp
)

ament_target_dependencies(shape_estimation_lib ${SHAPE_ESTIMATION_DEPENDENCIES})

target_include_directories(shape_estimation_lib
  SYSTEM PUBLIC
  "${PCL_INCLUDE_DIRS}"
  "${EIGEN3_INCLUDE_DIR}"
)

ament_auto_add_library(shape_estimation_node SHARED
  src/node.cpp
)

target_include_directories(shape_estimation_node
  PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(shape_estimation_node ${SHAPE_ESTIMATION_DEPENDENCIES})

target_link_libraries(shape_estimation_node
  shape_estimation_lib
)

rclcpp_components_register_node(shape_estimation_node
  PLUGIN "ShapeEstimationNode"
  EXECUTABLE shape_estimation
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
