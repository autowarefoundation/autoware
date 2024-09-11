cmake_minimum_required(VERSION 3.8)
project(autoware_cluster_merger)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/cluster_merger_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::cluster_merger::ClusterMergerNode"
  EXECUTABLE cluster_merger_node)


if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_auto_add_gtest(cluster_merger_tests
    test/test_cluster_merger.cpp
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
