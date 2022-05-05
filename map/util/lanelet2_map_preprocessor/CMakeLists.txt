cmake_minimum_required(VERSION 3.14)
project(lanelet2_map_preprocessor)

find_package(autoware_cmake REQUIRED)
autoware_package()

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
