cmake_minimum_required(VERSION 3.14)
project(pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)

ament_auto_add_executable(pose_initializer_node
  src/pose_initializer/pose_initializer_node.cpp
  src/pose_initializer/pose_initializer_core.cpp
  src/pose_initializer/gnss_module.cpp
  src/pose_initializer/ndt_module.cpp
  src/pose_initializer/stop_check_module.cpp
  src/pose_initializer/localization_trigger_module.cpp
)
ament_auto_add_executable(map_height_fitter
  src/map_height_fitter/map_height_fitter_node.cpp
  src/map_height_fitter/map_height_fitter_core.cpp
)
target_link_libraries(map_height_fitter ${PCL_LIBRARIES})

if(BUILD_TESTING)
  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})
    ament_add_gmock(${test_name} ${filepath})
    target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${test_name} fmt)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_copy_vector_to_array.cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
