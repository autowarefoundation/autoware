cmake_minimum_required(VERSION 3.14)
project(pose_estimator_arbiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(glog REQUIRED)

find_package(PCL REQUIRED COMPONENTS common)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

# ==============================
# switch rule library
ament_auto_add_library(switch_rule
  SHARED
  src/pose_estimator_arbiter/switch_rule/enable_all_rule.cpp
)
target_include_directories(switch_rule PUBLIC src)

# ==============================
# pose estimator arbiter node
ament_auto_add_executable(${PROJECT_NAME}
  src/pose_estimator_arbiter/pose_estimator_arbiter_core.cpp
  src/pose_estimator_arbiter/pose_estimator_arbiter_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC src)
target_link_libraries(${PROJECT_NAME} switch_rule glog::glog)

# ==============================
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
ament_auto_package(INSTALL_TO_SHARE
  launch
)
