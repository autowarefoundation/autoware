cmake_minimum_required(VERSION 3.8)
project(radar_object_tracker)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED) # for debug
find_package(glog REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(radar_object_tracker_utils SHARED
  src/utils/utils.cpp
  src/utils/radar_object_tracker_utils.cpp
)

ament_auto_add_library(radar_object_tracker_node SHARED
  src/radar_object_tracker_node/radar_object_tracker_node.cpp
  src/tracker/model/tracker_base.cpp
  src/tracker/model/linear_motion_tracker.cpp
  src/tracker/model/constant_turn_rate_motion_tracker.cpp
  src/data_association/data_association.cpp
  src/data_association/mu_successive_shortest_path/mu_successive_shortest_path_wrapper.cpp
)

target_link_libraries(radar_object_tracker_node
  Eigen3::Eigen
  yaml-cpp
  nlohmann_json::nlohmann_json # for debug
  glog::glog
  radar_object_tracker_utils
)

rclcpp_components_register_node(radar_object_tracker_node
  PLUGIN "RadarObjectTrackerNode"
  EXECUTABLE radar_object_tracker
)

# testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_radar_object_tracker_utils
    test/test_radar_object_tracker_utils.cpp
    src/utils/radar_object_tracker_utils.cpp
  )
  target_include_directories(test_radar_object_tracker_utils PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_radar_object_tracker_utils
    radar_object_tracker_utils
  )
endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
