cmake_minimum_required(VERSION 3.5)
project(detected_object_feature_remover)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate exe file
ament_auto_add_library(detected_object_feature_remover_node SHARED
  src/detected_object_feature_remover.cpp
)

rclcpp_components_register_node(detected_object_feature_remover_node
  PLUGIN "detected_object_feature_remover::DetectedObjectFeatureRemover"
  EXECUTABLE detected_object_feature_remover
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
