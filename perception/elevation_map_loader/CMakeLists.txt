cmake_minimum_required(VERSION 3.14)
project(elevation_map_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS io)

ament_auto_add_library(elevation_map_loader_node SHARED
  src/elevation_map_loader_node.cpp
)
target_link_libraries(elevation_map_loader_node ${PCL_LIBRARIES})

# TODO(wep21): workaround for iron.
# remove this block and update package.xml after iron.
find_package(rosbag2_storage_sqlite3)
target_include_directories(elevation_map_loader_node
  PRIVATE
    ${rosbag2_storage_sqlite3_INCLUDE_DIRS}
)

rclcpp_components_register_node(elevation_map_loader_node
  PLUGIN "ElevationMapLoaderNode"
  EXECUTABLE elevation_map_loader
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  data
)
