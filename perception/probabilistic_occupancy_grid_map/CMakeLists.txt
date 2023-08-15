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

ament_auto_add_library(${PROJECT_NAME}_common SHARED
  src/updater/occupancy_grid_map_binary_bayes_filter_updater.cpp
  src/utils/utils.cpp
)
target_link_libraries(${PROJECT_NAME}_common
  ${PCL_LIBRARIES}
)

# PointcloudBasedOccupancyGridMap
ament_auto_add_library(pointcloud_based_occupancy_grid_map SHARED
  src/pointcloud_based_occupancy_grid_map/pointcloud_based_occupancy_grid_map_node.cpp
  src/pointcloud_based_occupancy_grid_map/occupancy_grid_map_base.cpp
  src/pointcloud_based_occupancy_grid_map/occupancy_grid_map_fixed.cpp
  src/pointcloud_based_occupancy_grid_map/occupancy_grid_map_projective.cpp
)

target_link_libraries(pointcloud_based_occupancy_grid_map
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_common
)

rclcpp_components_register_node(pointcloud_based_occupancy_grid_map
  PLUGIN "occupancy_grid_map::PointcloudBasedOccupancyGridMapNode"
  EXECUTABLE pointcloud_based_occupancy_grid_map_node
)

# LaserscanBasedOccupancyGridMap
ament_auto_add_library(laserscan_based_occupancy_grid_map SHARED
  src/laserscan_based_occupancy_grid_map/laserscan_based_occupancy_grid_map_node.cpp
  src/laserscan_based_occupancy_grid_map/occupancy_grid_map.cpp
)

target_link_libraries(laserscan_based_occupancy_grid_map
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_common
)

rclcpp_components_register_node(laserscan_based_occupancy_grid_map
  PLUGIN "occupancy_grid_map::LaserscanBasedOccupancyGridMapNode"
  EXECUTABLE laserscan_based_occupancy_grid_map_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)

# test
if(BUILD_TESTING)
  # launch_testing
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_pointcloud_based_method.py)

  # gtest
  ament_add_gtest(test_utils
  test/test_utils.cpp
  )
  target_link_libraries(test_utils
    ${PCL_LIBRARIES}
    ${PROJECT_NAME}_common
  )
endif()
