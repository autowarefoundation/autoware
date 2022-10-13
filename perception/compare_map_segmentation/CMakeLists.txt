cmake_minimum_required(VERSION 3.14)
project(compare_map_segmentation)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP)

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GRID_MAP_INCLUDE_DIR}
)

add_library(compare_map_segmentation SHARED
  src/distance_based_compare_map_filter_nodelet.cpp
  src/voxel_based_approximate_compare_map_filter_nodelet.cpp
  src/voxel_based_compare_map_filter_nodelet.cpp
  src/voxel_distance_based_compare_map_filter_nodelet.cpp
  src/compare_elevation_map_filter_node.cpp
)

target_link_libraries(compare_map_segmentation
  pointcloud_preprocessor::pointcloud_preprocessor_filter_base
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(compare_map_segmentation
  grid_map_pcl
  grid_map_ros
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  tier4_autoware_utils
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

install(
  TARGETS compare_map_segmentation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
