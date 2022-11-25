cmake_minimum_required(VERSION 3.5)
project(front_vehicle_velocity_estimator)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(front_vehicle_velocity_estimator_node_component SHARED
  src/front_vehicle_velocity_estimator_node/front_vehicle_velocity_estimator_node.cpp
  src/front_vehicle_velocity_estimator_node/front_vehicle_velocity_estimator.cpp
)

rclcpp_components_register_node(front_vehicle_velocity_estimator_node_component
  PLUGIN "front_vehicle_velocity_estimator::FrontVehicleVelocityEstimatorNode"
  EXECUTABLE front_vehicle_velocity_estimator_node
)

# Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
