cmake_minimum_required(VERSION 3.14)
project(imu_corrector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(gyro_bias_estimation_module SHARED
  src/gyro_bias_estimation_module.cpp
)

ament_auto_add_library(imu_corrector_node SHARED
  src/imu_corrector_core.cpp
)

ament_auto_add_library(gyro_bias_estimator_node SHARED
  src/gyro_bias_estimator.cpp
)

target_link_libraries(gyro_bias_estimator_node gyro_bias_estimation_module)

rclcpp_components_register_node(imu_corrector_node
  PLUGIN "imu_corrector::ImuCorrector"
  EXECUTABLE imu_corrector
)

rclcpp_components_register_node(gyro_bias_estimator_node
  PLUGIN "imu_corrector::GyroBiasEstimator"
  EXECUTABLE gyro_bias_estimator
)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})
  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries("${test_name}" gyro_bias_estimation_module imu_corrector_node gyro_bias_estimator_node)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_gyro_bias_estimation_module.cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
