cmake_minimum_required(VERSION 3.5)
project(scenario_selector)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Target

## Target executable
set(SCENARIO_SELECTOR_SRC
  src/scenario_selector_node/scenario_selector_node.cpp
)

## scenario_selector_node
ament_auto_add_library(scenario_selector_node SHARED
  ${SCENARIO_SELECTOR_SRC}
)

rclcpp_components_register_node(scenario_selector_node
  PLUGIN "ScenarioSelectorNode"
  EXECUTABLE scenario_selector
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
