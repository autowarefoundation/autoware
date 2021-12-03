cmake_minimum_required(VERSION 3.5)
project(fault_injection)

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Targets
ament_auto_add_library(fault_injection_node_component SHARED
  src/fault_injection_node/fault_injection_node.cpp
)

rclcpp_components_register_node(fault_injection_node_component
  PLUGIN "fault_injection::FaultInjectionNode"
  EXECUTABLE fault_injection_node
)

## Tests
if(BUILD_TESTING)
  # Linter
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  ament_add_gtest(test_fault_injection_node_component
    test/src/main.cpp
    test/src/test_diagnostic_storage.cpp
  )

  target_link_libraries(test_fault_injection_node_component
    fault_injection_node_component
  )

  # TODO: Comment out until the error in CI is resolved.
  # launch_testing
  # find_package(launch_testing_ament_cmake REQUIRED)
  # add_launch_test(test/test_fault_injection_node.test.py)

  # install(DIRECTORY
  #   test/config
  #   test/launch
  #   DESTINATION share/${PROJECT_NAME}/
  # )
endif()

## Package
ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
