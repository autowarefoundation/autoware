cmake_minimum_required(VERSION 3.5)
project(object_range_splitter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

include_directories(
  include
)

ament_auto_add_library(object_range_splitter SHARED
  src/node.cpp
)

rclcpp_components_register_node(object_range_splitter
  PLUGIN "object_range_splitter::ObjectRangeSplitterNode"
  EXECUTABLE object_range_splitter_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
