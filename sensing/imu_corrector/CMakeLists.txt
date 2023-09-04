cmake_minimum_required(VERSION 3.14)
project(imu_corrector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(gyro_bias_estimation_module SHARED
  src/gyro_bias_estimation_module.cpp
)

ament_auto_add_executable(imu_corrector
  src/imu_corrector_core.cpp
  src/imu_corrector_node.cpp
)

ament_auto_add_executable(gyro_bias_estimator
  src/gyro_bias_estimator.cpp
  src/gyro_bias_estimator_node.cpp
)

target_link_libraries(gyro_bias_estimator gyro_bias_estimation_module)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})
  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries("${test_name}" gyro_bias_estimation_module)
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
