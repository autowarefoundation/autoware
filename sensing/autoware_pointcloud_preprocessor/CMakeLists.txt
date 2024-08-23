cmake_minimum_required(VERSION 3.14)
project(autoware_pointcloud_preprocessor)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Suppress CGAL warning
set(CGAL_DATA_DIR ".")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(pointcloud_preprocessor_filter_base SHARED
  src/filter.cpp
  src/utility/memory.cpp
)

target_include_directories(pointcloud_preprocessor_filter_base PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${autoware_point_types_INCLUDE_DIRS}
)

ament_target_dependencies(pointcloud_preprocessor_filter_base
  message_filters
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
  autoware_universe_utils
  pcl_ros
  autoware_point_types
)

add_library(faster_voxel_grid_downsample_filter SHARED
  src/downsample_filter/faster_voxel_grid_downsample_filter.cpp
)

target_include_directories(faster_voxel_grid_downsample_filter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(faster_voxel_grid_downsample_filter
  pcl_conversions
  rclcpp
  sensor_msgs
)

ament_auto_add_library(pointcloud_preprocessor_filter SHARED
  src/concatenate_data/concatenate_and_time_sync_nodelet.cpp
  src/concatenate_data/concatenate_pointclouds.cpp
  src/time_synchronizer/time_synchronizer_node.cpp
  src/crop_box_filter/crop_box_filter_nodelet.cpp
  src/downsample_filter/voxel_grid_downsample_filter_node.cpp
  src/downsample_filter/random_downsample_filter_nodelet.cpp
  src/downsample_filter/approximate_downsample_filter_nodelet.cpp
  src/downsample_filter/pickup_based_voxel_grid_downsample_filter.cpp
  src/outlier_filter/ring_outlier_filter_nodelet.cpp
  src/outlier_filter/voxel_grid_outlier_filter_node.cpp
  src/outlier_filter/radius_search_2d_outlier_filter_nodelet.cpp
  src/outlier_filter/dual_return_outlier_filter_nodelet.cpp
  src/passthrough_filter/passthrough_filter_nodelet.cpp
  src/passthrough_filter/passthrough_filter_uint16_nodelet.cpp
  src/passthrough_filter/passthrough_uint16.cpp
  src/pointcloud_accumulator/pointcloud_accumulator_nodelet.cpp
  src/vector_map_filter/lanelet2_map_filter_node.cpp
  src/distortion_corrector/distortion_corrector.cpp
  src/distortion_corrector/distortion_corrector_node.cpp
  src/blockage_diag/blockage_diag_node.cpp
  src/polygon_remover/polygon_remover.cpp
  src/vector_map_filter/vector_map_inside_area_filter_node.cpp
  src/utility/geometry.cpp
)

target_link_libraries(pointcloud_preprocessor_filter
  pointcloud_preprocessor_filter_base
  faster_voxel_grid_downsample_filter
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ========== Time synchronizer ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::PointCloudDataSynchronizerComponent"
  EXECUTABLE time_synchronizer_node)

# ========== Concatenate data ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::PointCloudConcatenationComponent"
  EXECUTABLE concatenate_pointclouds_node)

# ========== Concatenate and Sync data ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent"
  EXECUTABLE concatenate_data_node)

# ========== CropBox ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::CropBoxFilterComponent"
  EXECUTABLE crop_box_filter_node)

# ========== Down Sampler Filter ==========
# -- Voxel Grid Downsample Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::VoxelGridDownsampleFilterComponent"
  EXECUTABLE voxel_grid_downsample_filter_node)

# -- Pickup Based Voxel Grid Downsample Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::PickupBasedVoxelGridDownsampleFilterComponent"
  EXECUTABLE pickup_based_voxel_grid_downsample_filter_node)

# -- Random Downsample Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::RandomDownsampleFilterComponent"
  EXECUTABLE random_downsample_filter_node)

rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::ApproximateDownsampleFilterComponent"
  EXECUTABLE approximate_downsample_filter_node)

# ========== Outlier Filter ==========
# -- Ring Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::RingOutlierFilterComponent"
  EXECUTABLE ring_outlier_filter_node)

# -- Voxel Grid Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::VoxelGridOutlierFilterComponent"
  EXECUTABLE voxel_grid_outlier_filter_node)

# -- Radius Search 2D Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::RadiusSearch2DOutlierFilterComponent"
  EXECUTABLE radius_search_2d_outlier_filter_node)

# -- DualReturn Outlier Filter--
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::DualReturnOutlierFilterComponent"
  EXECUTABLE dual_return_outlier_filter_node)

# ========== Passthrough Filter ==========
# -- Passthrough Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::PassThroughFilterComponent"
  EXECUTABLE passthrough_filter_node)

# -- Passthrough Filter Uint16 --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::PassThroughFilterUInt16Component"
  EXECUTABLE passthrough_filter_uint16_node)

# ========== Pointcloud Accumulator Filter ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::PointcloudAccumulatorComponent"
  EXECUTABLE pointcloud_accumulator_node)

# ========== Vector Map Filter ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::Lanelet2MapFilterComponent"
  EXECUTABLE vector_map_filter_node)

# ========== Distortion Corrector ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::DistortionCorrectorComponent"
  EXECUTABLE distortion_corrector_node)

# ========== Blockage Diagnostics ===========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::BlockageDiagComponent"
  EXECUTABLE blockage_diag_node)

# ========== PolygonRemover ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::PolygonRemoverComponent"
  EXECUTABLE polygon_remover_node)

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
target_link_libraries(polygon_remover_node gmp CGAL CGAL::CGAL CGAL::CGAL_Core)

# ========== Vector Map Inside Area Filter ===========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "autoware::pointcloud_preprocessor::VectorMapInsideAreaFilterComponent"
  EXECUTABLE vector_map_inside_area_filter_node)

install(
  TARGETS pointcloud_preprocessor_filter_base EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS faster_voxel_grid_downsample_filter EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_utilities
    test/test_utilities.cpp
  )

  ament_add_gtest(test_distortion_corrector_node
    test/test_distortion_corrector_node.cpp
  )

  target_link_libraries(test_utilities pointcloud_preprocessor_filter)
  target_link_libraries(test_distortion_corrector_node pointcloud_preprocessor_filter)


endif()
