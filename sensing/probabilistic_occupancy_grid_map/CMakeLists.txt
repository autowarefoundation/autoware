cmake_minimum_required(VERSION 3.5)
project(probabilistic_occupancy_grid_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
ament_auto_find_build_dependencies()

# PointcloudBasedOccupancyGridMap
ament_auto_add_library(pointcloud_based_occupancy_grid_map SHARED
  src/pointcloud_based_occupancy_grid_map/pointcloud_based_occupancy_grid_map_node.cpp
  src/pointcloud_based_occupancy_grid_map/occupancy_grid_map.cpp
  src/updater/occupancy_grid_map_binary_bayes_filter_updater.cpp
)

target_link_libraries(pointcloud_based_occupancy_grid_map
  ${PCL_LIBRARIES}
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(pointcloud_based_occupancy_grid_map PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    # To avoid conflicts between cpplint and uncrustify w.r.t. inclusion guards
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
