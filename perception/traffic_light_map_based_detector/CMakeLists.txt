cmake_minimum_required(VERSION 3.5)
project(traffic_light_map_based_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(traffic_light_map_based_detector SHARED
  src/node.cpp
)

rclcpp_components_register_node(traffic_light_map_based_detector
  PLUGIN "traffic_light::MapBasedDetector"
  EXECUTABLE traffic_light_map_based_detector_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
