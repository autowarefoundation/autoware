cmake_minimum_required(VERSION 3.14)
project(ekf_localizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_find_build_dependencies()

ament_auto_add_library(ekf_localizer_lib SHARED
  src/ekf_localizer.cpp
  src/covariance.cpp
  src/diagnostics.cpp
  src/mahalanobis.cpp
  src/measurement.cpp
  src/state_transition.cpp
  src/warning_message.cpp
  src/ekf_module.cpp
)

target_link_libraries(ekf_localizer_lib Eigen3::Eigen)

ament_auto_add_executable(ekf_localizer src/ekf_localizer_node.cpp)

target_compile_options(ekf_localizer PUBLIC -g -Wall -Wextra -Wpedantic -Werror)

target_link_libraries(ekf_localizer ekf_localizer_lib)
target_include_directories(ekf_localizer PUBLIC include)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gtest(${test_name} ${filepath})
  target_link_libraries("${test_name}" ekf_localizer_lib)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  add_launch_test(
    test/test_ekf_localizer_launch.py
    TIMEOUT "30"
  )
  add_launch_test(
    test/test_ekf_localizer_mahalanobis.py
    TIMEOUT "30"
  )
  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB_RECURSE TEST_FILES test/*.cpp)

  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()


# if(BUILD_TESTING)
#   find_package(ament_cmake_ros REQUIRED)
#   ament_add_ros_isolated_gtest(ekf_localizer-test test/test_ekf_localizer.test
#     test/src/test_ekf_localizer.cpp
#     src/ekf_localizer.cpp
#     src/kalman_filter/kalman_filter.cpp
#     src/kalman_filter/time_delay_kalman_filter.cpp
#   )
#   target_include_directories(ekf_localizer-test
#     PRIVATE
#     include
#   )
#   ament_target_dependencies(ekf_localizer-test geometry_msgs rclcpp tf2 tf2_ros)
# endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
