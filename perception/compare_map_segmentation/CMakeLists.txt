cmake_minimum_required(VERSION 3.5)
project(compare_map_segmentation)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Ignore PCL errors in Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP)
ament_auto_find_build_dependencies()


###########
## Build ##
###########

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GRID_MAP_INCLUDE_DIR}
)

ament_auto_add_library(compare_map_segmentation SHARED
  src/distance_based_compare_map_filter_nodelet.cpp
  src/voxel_based_approximate_compare_map_filter_nodelet.cpp
  src/voxel_based_compare_map_filter_nodelet.cpp
  src/voxel_distance_based_compare_map_filter_nodelet.cpp
  src/compare_elevation_map_filter_node.cpp
)

target_link_libraries(compare_map_segmentation
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(OPENMP_FOUND)
  set_target_properties(compare_map_segmentation PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

# ========== Compare Map Filter ==========
# -- Distance Based Compare Map Filter --
rclcpp_components_register_node(compare_map_segmentation
  PLUGIN "compare_map_segmentation::DistanceBasedCompareMapFilterComponent"
  EXECUTABLE distance_based_compare_map_filter_node)

# -- Voxel Based Approximate Compare Map Filter --
rclcpp_components_register_node(compare_map_segmentation
  PLUGIN "compare_map_segmentation::VoxelBasedApproximateCompareMapFilterComponent"
  EXECUTABLE voxel_based_approximate_compare_map_filter_node)

# -- Voxel Based Compare Map Filter --
rclcpp_components_register_node(compare_map_segmentation
  PLUGIN "compare_map_segmentation::VoxelBasedCompareMapFilterComponent"
  EXECUTABLE voxel_based_compare_map_filter_node)

# -- Voxel Distance Based Compare Map Filter --
rclcpp_components_register_node(compare_map_segmentation
  PLUGIN "compare_map_segmentation::VoxelDistanceBasedCompareMapFilterComponent"
  EXECUTABLE voxel_distance_based_compare_map_filter_node)

# -- Compare Elevation Map Filter --
rclcpp_components_register_node(compare_map_segmentation
  PLUGIN "compare_map_segmentation::CompareElevationMapFilterComponent"
  EXECUTABLE compare_elevation_map_filter_node)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
