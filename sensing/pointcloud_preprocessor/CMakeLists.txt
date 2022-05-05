cmake_minimum_required(VERSION 3.14)
project(pointcloud_preprocessor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
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

ament_auto_add_library(pointcloud_preprocessor_filter SHARED
  src/filter.cpp
  src/concatenate_data/concatenate_data_nodelet.cpp
  src/crop_box_filter/crop_box_filter_nodelet.cpp
  src/downsample_filter/voxel_grid_downsample_filter_nodelet.cpp
  src/downsample_filter/random_downsample_filter_nodelet.cpp
  src/downsample_filter/approximate_downsample_filter_nodelet.cpp
  src/outlier_filter/ring_outlier_filter_nodelet.cpp
  src/outlier_filter/voxel_grid_outlier_filter_nodelet.cpp
  src/outlier_filter/radius_search_2d_outlier_filter_nodelet.cpp
  src/outlier_filter/dual_return_outlier_filter_nodelet.cpp
  src/passthrough_filter/passthrough_filter_nodelet.cpp
  src/passthrough_filter/passthrough_filter_uint16_nodelet.cpp
  src/passthrough_filter/passthrough_uint16.cpp
  src/pointcloud_accumulator/pointcloud_accumulator_nodelet.cpp
  src/vector_map_filter/lanelet2_map_filter_nodelet.cpp
  src/distortion_corrector/distortion_corrector.cpp
  src/blockage_diag/blockage_diag_nodelet.cpp
)

target_link_libraries(pointcloud_preprocessor_filter
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(OPENMP_FOUND)
  set_target_properties(pointcloud_preprocessor_filter PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

# ========== Concatenate data ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent"
  EXECUTABLE concatenate_data_node)


# ========== CropBox ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::CropBoxFilterComponent"
  EXECUTABLE crop_box_filter_node)


# ========== Down Sampler Filter ==========
# -- Voxel Grid Downsample Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::VoxelGridDownsampleFilterComponent"
  EXECUTABLE voxel_grid_downsample_filter_node)

# -- Random Downsample Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::RandomDownsampleFilterComponent"
  EXECUTABLE random_downsample_filter_node)

rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::ApproximateDownsampleFilterComponent"
  EXECUTABLE approximate_downsample_filter_node)

# ========== Outlier Filter ==========
# -- Ring Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::RingOutlierFilterComponent"
  EXECUTABLE ring_outlier_filter_node)

# -- Voxel Grid Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::VoxelGridOutlierFilterComponent"
  EXECUTABLE voxel_grid_outlier_filter_node)

# -- Radius Search 2D Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::RadiusSearch2DOutlierFilterComponent"
  EXECUTABLE radius_search_2d_outlier_filter_node)

# -- DualReturn Outlier Filter--
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::DualReturnOutlierFilterComponent"
  EXECUTABLE dual_return_outlier_filter_node)

# ========== Passthrough Filter ==========
# -- Passthrough Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::PassThroughFilterComponent"
  EXECUTABLE passthrough_filter_node)

# -- Passthrough Filter Uint16 --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::PassThroughFilterUInt16Component"
  EXECUTABLE passthrough_filter_uint16_node)


# ========== Pointcloud Accumulator Filter ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::PointcloudAccumulatorComponent"
  EXECUTABLE pointcloud_accumulator_node)


# ========== Vector Map Filter ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::Lanelet2MapFilterComponent"
  EXECUTABLE vector_map_filter_node)


# ========== Distortion Corrector ==========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::DistortionCorrectorComponent"
  EXECUTABLE distortion_corrector_node)

# ========== Blockage Diagnostics ===========
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::BlockageDiagComponent"
  EXECUTABLE blockage_diag_node)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
