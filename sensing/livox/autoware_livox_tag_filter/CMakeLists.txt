cmake_minimum_required(VERSION 3.14)
project(autoware_livox_tag_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_tag_filter_node.cpp
  src/livox_tag_filter_node.hpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::livox_tag_filter::LivoxTagFilterNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
