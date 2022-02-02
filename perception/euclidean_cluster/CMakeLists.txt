cmake_minimum_required(VERSION 3.5)
project(euclidean_cluster)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common search filters segmentation)
ament_auto_find_build_dependencies()


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
