cmake_minimum_required(VERSION 3.5)
project(detected_object_validation)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

### Find Packages
find_package(ament_cmake_auto REQUIRED)

### Find OpenCV Dependencies
find_package(OpenCV REQUIRED)

### Find PCL Dependencies
find_package(PCL REQUIRED QUIET COMPONENTS common search filters segmentation)

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

### Find dependencies listed in the package.xml
ament_auto_find_build_dependencies()

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
