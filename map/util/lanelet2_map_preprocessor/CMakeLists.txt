cmake_minimum_required(VERSION 3.5)
project(lanelet2_map_preprocessor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  SYSTEM
    ${PCL_INCLUDE_DIRS}
)

ament_auto_add_executable(fix_z_value_by_pcd src/fix_z_value_by_pcd.cpp)
ament_auto_add_executable(transform_maps src/transform_maps.cpp)
ament_auto_add_executable(merge_close_lines src/merge_close_lines.cpp)
ament_auto_add_executable(merge_close_points src/merge_close_points.cpp)
ament_auto_add_executable(remove_unreferenced_geometry src/remove_unreferenced_geometry.cpp)
ament_auto_add_executable(fix_lane_change_tags src/fix_lane_change_tags.cpp)

ament_auto_package()
