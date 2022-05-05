cmake_minimum_required(VERSION 3.14)
project(detected_object_validation)

find_package(autoware_cmake REQUIRED)
autoware_package()

### Find OpenCV Dependencies
find_package(OpenCV REQUIRED)

### Find PCL Dependencies
find_package(PCL REQUIRED QUIET COMPONENTS common search filters segmentation)

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Generate occupancy grid based validator exe file
set(OCCUPANCY_GRID_BASED_VALIDATOR_SRC
  src/occupancy_grid_based_validator.cpp
)

ament_auto_add_library(occupancy_grid_based_validator SHARED
  ${OCCUPANCY_GRID_BASED_VALIDATOR_SRC}
)

target_link_libraries(occupancy_grid_based_validator
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

# Generate obstacle pointcloud based validator exe file
set(OBSTACLE_POINTCLOUD_BASED_VALIDATOR_SRC
  src/obstacle_pointcloud_based_validator.cpp
)

ament_auto_add_library(obstacle_pointcloud_based_validator SHARED
  ${OBSTACLE_POINTCLOUD_BASED_VALIDATOR_SRC}
)

target_link_libraries(obstacle_pointcloud_based_validator
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

rclcpp_components_register_node(obstacle_pointcloud_based_validator
  PLUGIN "obstacle_pointcloud_based_validator::ObstaclePointCloudBasedValidator"
  EXECUTABLE obstacle_pointcloud_based_validator_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
