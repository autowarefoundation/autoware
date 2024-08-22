cmake_minimum_required(VERSION 3.14)
project(autoware_pose_estimator_arbiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)
include_directories(
  SYSTEM ${PCL_INCLUDE_DIRS}
  src
)

# ==============================
# pose estimator arbiter node
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_estimator_arbiter_core.cpp
  src/switch_rule/enable_all_rule.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC src)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose_estimator_arbiter::PoseEstimatorArbiter"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # define test definition macro
  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})
    ament_add_gmock(${test_name} ${filepath})
    target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(${test_name} fmt)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  add_testcase(test/test_shared_data.cpp)
  add_ros_test(
    test/test_pose_estimator_arbiter.py
    TIMEOUT "30"
  )
endif()

# ==============================
# In practice, the example rule is not used as autoware code.
# It exists only for user reference and is tested only.
add_subdirectory(example_rule)

# ==============================
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
