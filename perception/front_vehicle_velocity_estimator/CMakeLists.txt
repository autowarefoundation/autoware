cmake_minimum_required(VERSION 3.5)
project(front_vehicle_velocity_estimator)

# Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
