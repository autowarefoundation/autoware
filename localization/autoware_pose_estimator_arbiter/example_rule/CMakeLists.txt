# example switch rule library
ament_auto_add_library(example_rule
  SHARED
  src/switch_rule/pcd_map_based_rule.cpp
  src/switch_rule/vector_map_based_rule.cpp
  src/rule_helper/pcd_occupancy.cpp
  src/rule_helper/pose_estimator_area.cpp
)
target_include_directories(example_rule PUBLIC src example_rule/src)
target_link_libraries(example_rule autoware_pose_estimator_arbiter)

# ==============================
# define test definition macro
function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gtest(${test_name} ${filepath})
  target_link_libraries("${test_name}" autoware_pose_estimator_arbiter example_rule)
  target_include_directories(${test_name} PUBLIC src)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

# ==============================
# build test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_testcase(test/test_rule_helper.cpp)
  add_testcase(test/test_vector_map_based_rule.cpp)
  add_testcase(test/test_pcd_map_based_rule.cpp)
endif()
