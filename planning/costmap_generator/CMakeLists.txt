cmake_minimum_required(VERSION 3.5)

project(costmap_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(FLANN REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${GRID_MAP_INCLUDE_DIR}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(costmap_generator_lib SHARED
  nodes/costmap_generator/points_to_costmap.cpp
  nodes/costmap_generator/objects_to_costmap.cpp
  nodes/costmap_generator/object_map_utils.cpp
)
target_link_libraries(costmap_generator_lib
  ${PCL_LIBRARIES}
  FLANN::FLANN
)

ament_auto_add_library(costmap_generator_node SHARED
  nodes/costmap_generator/costmap_generator_node.cpp
)
target_link_libraries(costmap_generator_node
  ${PCL_LIBRARIES}
  costmap_generator_lib
)

rclcpp_components_register_node(costmap_generator_node
  PLUGIN "CostmapGenerator"
  EXECUTABLE costmap_generator
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
