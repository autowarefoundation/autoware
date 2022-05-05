cmake_minimum_required(VERSION 3.14)
project(map_tf_generator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

ament_auto_add_library(map_tf_generator_node SHARED
  src/map_tf_generator_node.cpp
)
target_link_libraries(map_tf_generator_node ${PCL_LIBRARIES})

rclcpp_components_register_node(map_tf_generator_node
  PLUGIN "MapTFGeneratorNode"
  EXECUTABLE map_tf_generator
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
