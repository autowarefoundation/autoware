cmake_minimum_required(VERSION 3.8)
project(autoware_radar_object_tracker)

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

ament_auto_add_library(${PROJECT_NAME}_utils SHARED
  src/utils/utils.cpp
  src/utils/radar_object_tracker_utils.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/radar_object_tracker_node.cpp
  src/tracker/model/tracker_base.cpp
  src/tracker/model/linear_motion_tracker.cpp
  src/tracker/model/constant_turn_rate_motion_tracker.cpp
  src/association/data_association.cpp
  src/association/mu_ssp/mu_successive_shortest_path_wrapper.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
  nlohmann_json::nlohmann_json # for debug
  glog::glog
  ${PROJECT_NAME}_utils
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::radar_object_tracker::RadarObjectTrackerNode"
  EXECUTABLE radar_object_tracker_node
)

# testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_${PROJECT_NAME}_utils
    test/test_radar_object_tracker_utils.cpp
    src/utils/radar_object_tracker_utils.cpp
  )
  target_include_directories(test_${PROJECT_NAME}_utils PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_${PROJECT_NAME}_utils
    ${PROJECT_NAME}_utils
  )
endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
