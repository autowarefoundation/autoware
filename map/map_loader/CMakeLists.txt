cmake_minimum_required(VERSION 3.5)
project(map_loader)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(PCL REQUIRED COMPONENTS io)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pointcloud_map_loader_node SHARED
  src/pointcloud_map_loader/pointcloud_map_loader_node.cpp
)
target_link_libraries(pointcloud_map_loader_node ${PCL_LIBRARIES})

rclcpp_components_register_node(pointcloud_map_loader_node
  PLUGIN "PointCloudMapLoaderNode"
  EXECUTABLE pointcloud_map_loader
)

ament_auto_add_library(lanelet2_map_loader_node SHARED
  src/lanelet2_map_loader/lanelet2_map_loader_node.cpp
)

rclcpp_components_register_node(lanelet2_map_loader_node
  PLUGIN "Lanelet2MapLoaderNode"
  EXECUTABLE lanelet2_map_loader
)

ament_auto_add_library(lanelet2_map_visualization_node SHARED
  src/lanelet2_map_loader/lanelet2_map_visualization_node.cpp
)

rclcpp_components_register_node(lanelet2_map_visualization_node
  PLUGIN "Lanelet2MapVisualizationNode"
  EXECUTABLE lanelet2_map_visualization
)

install(PROGRAMS
  script/map_hash_generator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
