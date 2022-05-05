cmake_minimum_required(VERSION 3.14)
project(map_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS io)

ament_auto_add_library(pointcloud_map_loader_node SHARED
  src/pointcloud_map_loader/pointcloud_map_loader_node.cpp
)
target_link_libraries(pointcloud_map_loader_node ${PCL_LIBRARIES})

target_include_directories(pointcloud_map_loader_node
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
)

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

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
