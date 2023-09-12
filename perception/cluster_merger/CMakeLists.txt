cmake_minimum_required(VERSION 3.8)
project(cluster_merger)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(cluster_merger_node_component SHARED
  src/cluster_merger/node.cpp
)

rclcpp_components_register_node(cluster_merger_node_component
  PLUGIN "cluster_merger::ClusterMergerNode"
  EXECUTABLE cluster_merger_node)


if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
