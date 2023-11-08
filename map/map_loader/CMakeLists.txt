cmake_minimum_required(VERSION 3.14)
project(map_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common io filters)

ament_auto_add_library(pointcloud_map_loader_node SHARED
  src/pointcloud_map_loader/pointcloud_map_loader_node.cpp
  src/pointcloud_map_loader/pointcloud_map_loader_module.cpp
  src/pointcloud_map_loader/partial_map_loader_module.cpp
  src/pointcloud_map_loader/differential_map_loader_module.cpp
  src/pointcloud_map_loader/selected_map_loader_module.cpp
  src/pointcloud_map_loader/utils.cpp
)
target_link_libraries(pointcloud_map_loader_node ${PCL_LIBRARIES})
target_link_libraries(pointcloud_map_loader_node yaml-cpp)

target_include_directories(pointcloud_map_loader_node
  SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
)

rclcpp_components_register_node(pointcloud_map_loader_node
  PLUGIN "PointCloudMapLoaderNode"
  EXECUTABLE pointcloud_map_loader
)

ament_auto_add_library(lanelet2_map_loader_node SHARED
  src/lanelet2_map_loader/lanelet2_map_loader_node.cpp
)

rclcpp_components_register_node(lanelet2_map_loader_node
  PLUGIN "Lanelet2MapLoaderNode"
  EXECUTABLE lanelet2_map_loader
)

ament_auto_add_library(lanelet2_map_visualization_node SHARED
  src/lanelet2_map_loader/lanelet2_map_visualization_node.cpp
)

rclcpp_components_register_node(lanelet2_map_visualization_node
  PLUGIN "Lanelet2MapVisualizationNode"
  EXECUTABLE lanelet2_map_visualization
)

if(BUILD_TESTING)
  add_launch_test(
    test/lanelet2_map_loader_launch.test.py
    TIMEOUT "30"
  )
  install(DIRECTORY
    test/data/
    DESTINATION share/${PROJECT_NAME}/test/data/
  )

  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})
    ament_add_gmock(${test_name} ${filepath})
    target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${test_name} ${${PROJECT_NAME}_LIBRARIES})
    target_link_libraries(${test_name} yaml-cpp)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_cylinder_box_overlap.cpp)
  add_testcase(test/test_replace_with_absolute_path.cpp)
  add_testcase(test/test_load_pcd_metadata.cpp)
  add_testcase(test/test_pointcloud_map_loader_module.cpp)
  add_testcase(test/test_partial_map_loader_module.cpp)
  add_testcase(test/test_differential_map_loader_module.cpp)
endif()

install(PROGRAMS
  script/map_hash_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
