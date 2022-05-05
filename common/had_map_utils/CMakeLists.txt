cmake_minimum_required(VERSION 3.14)
project(had_map_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/had_map_utils.cpp
  src/had_map_computation.cpp
  src/had_map_conversion.cpp
  src/had_map_query.cpp
  src/had_map_visualization.cpp)

# Disable warnings due to external dependencies
get_target_property(lanelet2_core_INCLUDE_DIR
  lanelet2_core::lanelet2_core INTERFACE_INCLUDE_DIRECTORIES
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${lanelet2_core_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
)

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
target_link_libraries(${PROJECT_NAME} CGAL CGAL::CGAL CGAL::CGAL_Core)

ament_auto_package()
