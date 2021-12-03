cmake_minimum_required(VERSION 3.5)
project(pointcloud_preprocessor)

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
  src/outlier_filter/occupancy_grid_map_outlier_filter_nodelet.cpp
  src/outlier_filter/dual_return_outlier_filter_nodelet.cpp
  src/passthrough_filter/passthrough_filter_nodelet.cpp
  src/passthrough_filter/passthrough_filter_uint16_nodelet.cpp
  src/passthrough_filter/passthrough_uint16.cpp
  src/pointcloud_accumulator/pointcloud_accumulator_nodelet.cpp
  src/vector_map_filter/lanelet2_map_filter_nodelet.cpp
  src/distortion_corrector/distortion_corrector.cpp
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

# -- Occupancy Grid Map Outlier Filter --
rclcpp_components_register_node(pointcloud_preprocessor_filter
  PLUGIN "pointcloud_preprocessor::OccupancyGridMapOutlierFilterComponent"
  EXECUTABLE occupancy_grid_map_outlier_filter_node)

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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
)
