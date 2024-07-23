cmake_minimum_required(VERSION 3.14)
project(autoware_detected_object_validation)

find_package(autoware_cmake REQUIRED)
autoware_package()

### Find OpenCV Dependencies
find_package(OpenCV REQUIRED)

### Find PCL Dependencies
find_package(PCL REQUIRED QUIET COMPONENTS common search filters segmentation)

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

### Find Boost Dependencies
find_package(Boost REQUIRED)

include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

# Generate occupancy grid based validator exe file
ament_auto_add_library(occupancy_grid_based_validator SHARED
  src/occupancy_grid_map/occupancy_grid_map_validator.cpp
)

target_link_libraries(occupancy_grid_based_validator
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

# Generate obstacle pointcloud based validator exe file
ament_auto_add_library(obstacle_pointcloud_based_validator SHARED
  src/obstacle_pointcloud/obstacle_pointcloud_validator.cpp
)

target_link_libraries(obstacle_pointcloud_based_validator
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_auto_add_library(object_lanelet_filter SHARED
  src/lanelet_filter/lanelet_filter.cpp
  lib/utils/utils.cpp
)

ament_auto_add_library(object_position_filter SHARED
  src/position_filter/position_filter.cpp
  lib/utils/utils.cpp
)

rclcpp_components_register_node(obstacle_pointcloud_based_validator
  PLUGIN "autoware::detected_object_validation::obstacle_pointcloud::ObstaclePointCloudBasedValidator"
  EXECUTABLE obstacle_pointcloud_based_validator_node
)

rclcpp_components_register_node(object_lanelet_filter
  PLUGIN "autoware::detected_object_validation::lanelet_filter::ObjectLaneletFilterNode"
  EXECUTABLE object_lanelet_filter_node
)

rclcpp_components_register_node(object_position_filter
  PLUGIN "autoware::detected_object_validation::position_filter::ObjectPositionFilterNode"
  EXECUTABLE object_position_filter_node
)

rclcpp_components_register_node(occupancy_grid_based_validator
  PLUGIN "autoware::detected_object_validation::occupancy_grid_map::OccupancyGridBasedValidator"
  EXECUTABLE occupancy_grid_based_validator_node
)

if(BUILD_TESTING)
  ament_auto_add_gtest(detection_object_validation_tests
    test/test_utils.cpp
    test/object_position_filter/test_object_position_filter.cpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
