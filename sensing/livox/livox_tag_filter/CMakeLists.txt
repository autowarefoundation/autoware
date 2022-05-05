cmake_minimum_required(VERSION 3.14)
project(livox_tag_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

ament_auto_add_library(livox_tag_filter SHARED
  src/livox_tag_filter_node/livox_tag_filter_node.cpp
)

target_link_libraries(livox_tag_filter
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(livox_tag_filter
  PLUGIN "livox_tag_filter::LivoxTagFilterNode"
  EXECUTABLE livox_tag_filter_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
