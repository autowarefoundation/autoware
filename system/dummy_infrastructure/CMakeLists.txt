cmake_minimum_required(VERSION 3.5)
project(dummy_infrastructure)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Targets
ament_auto_add_library(dummy_infrastructure_node_component SHARED
  src/dummy_infrastructure_node/dummy_infrastructure_node.cpp
)

rclcpp_components_register_node(dummy_infrastructure_node_component
  PLUGIN "dummy_infrastructure::DummyInfrastructureNode"
  EXECUTABLE dummy_infrastructure_node
)

## Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Package
ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
