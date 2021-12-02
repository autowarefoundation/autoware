cmake_minimum_required(VERSION 3.0.2)
project(lanelet2_map_preprocessor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  lanelet2_extension
  pcl_ros
  roscpp
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fix_z_value_by_pcd src/fix_z_value_by_pcd.cpp)
add_dependencies(fix_z_value_by_pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fix_z_value_by_pcd ${catkin_LIBRARIES})

add_executable(transform_maps src/transform_maps.cpp)
add_dependencies(transform_maps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_maps ${catkin_LIBRARIES})

add_executable(merge_close_lines src/merge_close_lines.cpp)
add_dependencies(merge_close_lines ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge_close_lines ${catkin_LIBRARIES})

add_executable(merge_close_points src/merge_close_points.cpp)
add_dependencies(merge_close_points ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge_close_points ${catkin_LIBRARIES})

add_executable(remove_unreferenced_geometry src/remove_unreferenced_geometry.cpp)
add_dependencies(remove_unreferenced_geometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(remove_unreferenced_geometry ${catkin_LIBRARIES})

add_executable(fix_lane_change_tags src/fix_lane_change_tags.cpp)
add_dependencies(fix_lane_change_tags ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fix_lane_change_tags ${catkin_LIBRARIES})

## Install executables and/or libraries
install(
  TARGETS
    fix_z_value_by_pcd
    transform_maps
    merge_close_lines
    merge_close_points
    remove_unreferenced_geometry
    fix_lane_change_tags
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
