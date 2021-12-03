cmake_minimum_required(VERSION 3.5)
project(livox_tag_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# Target
## livox_tag_filter_node
ament_auto_add_library(livox_tag_filter SHARED
  src/livox_tag_filter_node/livox_tag_filter_node.cpp
)

target_link_libraries(livox_tag_filter
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(livox_tag_filter
  PLUGIN "livox_tag_filter::LivoxTagFilterNode"
  EXECUTABLE livox_tag_filter_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
## executables and libraries
ament_auto_package(INSTALL_TO_SHARE
  launch
)
