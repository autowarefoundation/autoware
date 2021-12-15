# Copyright 2019 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_geometry)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# includes
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/geometry/spatial_hash.hpp
  include/geometry/intersection.hpp
  include/geometry/spatial_hash_config.hpp
  src/spatial_hash.cpp
  src/bounding_box.cpp)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # gtest
  set(GEOMETRY_GTEST geometry_gtest)
  set(GEOMETRY_SRC test/src/test_geometry.cpp
    test/src/test_convex_hull.cpp
    test/src/test_hull_pockets.cpp
    test/src/test_interval.cpp
    test/src/lookup_table.cpp
    test/src/test_area.cpp
    test/src/test_common_2d.cpp
    test/src/test_intersection.cpp
  )
  ament_add_gtest(${GEOMETRY_GTEST} ${GEOMETRY_SRC})
  autoware_set_compile_options(${GEOMETRY_GTEST})
  target_compile_options(${GEOMETRY_GTEST} PRIVATE -Wno-conversion -Wno-sign-conversion)
  target_include_directories(${GEOMETRY_GTEST} PRIVATE "test/include" "include")
  ament_target_dependencies(${GEOMETRY_GTEST}
    "autoware_auto_common"
    "autoware_auto_geometry_msgs"
    "autoware_auto_planning_msgs"
    "autoware_auto_vehicle_msgs"
    "geometry_msgs"
    "osrf_testing_tools_cpp")
  target_link_libraries(${GEOMETRY_GTEST} ${PROJECT_NAME})
endif()

# Ament Exporting
ament_auto_package()
