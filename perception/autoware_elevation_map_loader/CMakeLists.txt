cmake_minimum_required(VERSION 3.14)
project(autoware_elevation_map_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS io)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/elevation_map_loader_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# TODO(wep21): workaround for iron.
# remove this block and update package.xml after iron.
find_package(rosbag2_storage_sqlite3)
target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${rosbag2_storage_sqlite3_INCLUDE_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::elevation_map_loader::ElevationMapLoaderNode"
  EXECUTABLE elevation_map_loader_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  data
)
