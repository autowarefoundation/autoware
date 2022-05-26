cmake_minimum_required(VERSION 3.14)
project(probabilistic_occupancy_grid_map)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

# PointcloudBasedOccupancyGridMap
ament_auto_add_library(pointcloud_based_occupancy_grid_map SHARED
  src/pointcloud_based_occupancy_grid_map/pointcloud_based_occupancy_grid_map_node.cpp
  src/pointcloud_based_occupancy_grid_map/occupancy_grid_map.cpp
  src/updater/occupancy_grid_map_binary_bayes_filter_updater.cpp
)

target_link_libraries(pointcloud_based_occupancy_grid_map
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(pointcloud_based_occupancy_grid_map
  PLUGIN "occupancy_grid_map::PointcloudBasedOccupancyGridMapNode"
  EXECUTABLE pointcloud_based_occupancy_grid_map_node
)

# LaserscanBasedOccupancyGridMap
ament_auto_add_library(laserscan_based_occupancy_grid_map SHARED
  src/laserscan_based_occupancy_grid_map/laserscan_based_occupancy_grid_map_node.cpp
  src/laserscan_based_occupancy_grid_map/occupancy_grid_map.cpp
  src/updater/occupancy_grid_map_binary_bayes_filter_updater.cpp
)

target_link_libraries(laserscan_based_occupancy_grid_map
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(laserscan_based_occupancy_grid_map
  PLUGIN "occupancy_grid_map::LaserscanBasedOccupancyGridMapNode"
  EXECUTABLE laserscan_based_occupancy_grid_map_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
