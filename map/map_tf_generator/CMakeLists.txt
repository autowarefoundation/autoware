cmake_minimum_required(VERSION 3.5)
project(map_tf_generator)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

### ROS Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### non-ROS Dependencies
find_package(PCL REQUIRED)

# Generate exe file
ament_auto_add_library(map_tf_generator_node SHARED
  src/map_tf_generator_node.cpp
)
target_link_libraries(map_tf_generator_node ${PCL_LIBRARIES})

rclcpp_components_register_node(map_tf_generator_node
  PLUGIN "MapTFGeneratorNode"
  EXECUTABLE map_tf_generator
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Install
ament_auto_package(INSTALL_TO_SHARE
  launch
)
