cmake_minimum_required(VERSION 3.14)
project(autoware_map_projection_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map_projection_loader.cpp
  src/load_info_from_lanelet2_map.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::map_projection_loader::MapProjectionLoader"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

target_link_libraries(${PROJECT_NAME} yaml-cpp)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})
  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries("${test_name}" ${PROJECT_NAME})
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  # Test python scripts
  add_launch_test(
    test/test_node_load_mgrs_from_yaml.test.py
    TIMEOUT "30"
  )
  add_launch_test(
    test/test_node_load_local_from_yaml.test.py
    TIMEOUT "30"
  )
  add_launch_test(
    test/test_node_load_local_cartesian_utm_from_yaml.test.py
    TIMEOUT "30"
  )
  add_launch_test(
    test/test_node_load_transverse_mercator_from_yaml.test.py
    TIMEOUT "30"
  )
  install(DIRECTORY
    test/data/
    DESTINATION share/${PROJECT_NAME}/test/data/
  )

  # Test c++ scripts
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_load_info_from_lanelet2_map.cpp)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
