cmake_minimum_required(VERSION 3.5)
project(laserscan_to_occupancy_grid_map)

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

ament_auto_add_library(laserscan_to_occupancy_grid_map SHARED
  src/laserscan_to_occupancy_grid_map_node.cpp
  src/occupancy_grid_map.cpp
  src/updater/occupancy_grid_map_binary_bayes_filter_updater.cpp
)

target_link_libraries(laserscan_to_occupancy_grid_map
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(laserscan_to_occupancy_grid_map
  PLUGIN "occupancy_grid_map::OccupancyGridMapNode"
  EXECUTABLE laserscan_to_occupancy_grid_map_node
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
