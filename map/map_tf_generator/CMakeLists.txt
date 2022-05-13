cmake_minimum_required(VERSION 3.14)
project(map_tf_generator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

ament_auto_add_library(map_tf_generator_node SHARED
  src/map_tf_generator_node.cpp
)
target_link_libraries(map_tf_generator_node ${PCL_LIBRARIES})

rclcpp_components_register_node(map_tf_generator_node
  PLUGIN "MapTFGeneratorNode"
  EXECUTABLE map_tf_generator
)


if(BUILD_TESTING)
  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})

    ament_add_gmock(${test_name} ${filepath})
    target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_testcase(test/test_uniform_random.cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
