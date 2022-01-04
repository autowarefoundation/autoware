# Copyright 2020 TierIV, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(had_map_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/had_map_utils.cpp
  src/had_map_computation.cpp
  src/had_map_conversion.cpp
  src/had_map_query.cpp
  src/had_map_visualization.cpp)

# Disable warnings due to external dependencies (Eigen)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
)

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
target_link_libraries(${PROJECT_NAME} CGAL CGAL::CGAL CGAL::CGAL_Core)
autoware_set_compile_options(${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_auto_package()
