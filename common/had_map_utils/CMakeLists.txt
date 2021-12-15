# Copyright 2020 TierIV, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(had_map_utils)

find_package(ament_cmake_auto REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
#find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# Disable warnings due to external dependencies (Eigen)
#include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/had_map_utils/had_map_utils.hpp
  include/had_map_utils/had_map_computation.hpp
  include/had_map_utils/had_map_conversion.hpp
  include/had_map_utils/had_map_query.hpp
  include/had_map_utils/had_map_visualization.hpp
  include/had_map_utils/visibility_control.hpp
  src/had_map_utils.cpp
  src/had_map_computation.cpp
  src/had_map_conversion.cpp
  src/had_map_query.cpp
  src/had_map_visualization.cpp)

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
target_link_libraries(${PROJECT_NAME} CGAL CGAL::CGAL CGAL::CGAL_Core)
autoware_set_compile_options(${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_auto_package()
