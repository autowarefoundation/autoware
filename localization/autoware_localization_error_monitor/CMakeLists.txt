cmake_minimum_required(VERSION 3.14)
project(autoware_localization_error_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/localization_error_monitor.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::localization_error_monitor::LocalizationErrorMonitor"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})
    ament_add_gtest(${test_name} ${filepath})
    target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_diagnostics.cpp)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
