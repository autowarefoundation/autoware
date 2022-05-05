cmake_minimum_required(VERSION 3.14)
project(euclidean_cluster)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

include_directories(
  include
  SYSTEM
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(cluster_lib SHARED
  lib/utils.cpp
  lib/euclidean_cluster.cpp
  lib/voxel_grid_based_euclidean_cluster.cpp
)

target_link_libraries(cluster_lib
  ${PCL_LIBRARIES}
)

target_include_directories(cluster_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_auto_add_library(euclidean_cluster_node_core SHARED
  src/euclidean_cluster_node.cpp
)
target_link_libraries(euclidean_cluster_node_core
  ${PCL_LIBRARIES}
  cluster_lib
)

rclcpp_components_register_node(euclidean_cluster_node_core
  PLUGIN "euclidean_cluster::EuclideanClusterNode"
  EXECUTABLE euclidean_cluster_node
)

ament_auto_add_library(voxel_grid_based_euclidean_cluster_node_core SHARED
  src/voxel_grid_based_euclidean_cluster_node.cpp
)
target_link_libraries(voxel_grid_based_euclidean_cluster_node_core
  ${PCL_LIBRARIES}
  cluster_lib
)

rclcpp_components_register_node(voxel_grid_based_euclidean_cluster_node_core
  PLUGIN "euclidean_cluster::VoxelGridBasedEuclideanClusterNode"
  EXECUTABLE voxel_grid_based_euclidean_cluster_node
)

ament_auto_package(INSTALL_TO_SHARE
    launch
    config
)
