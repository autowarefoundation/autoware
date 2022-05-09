cmake_minimum_required(VERSION 3.14)
project(fake_test_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(fake_test_node SHARED src/fake_test_node.cpp)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_fake_test_node
    test/test_fake_test_node.cpp
  )
  add_dependencies(test_fake_test_node fake_test_node)
  target_link_libraries(test_fake_test_node fake_test_node)
endif()

ament_auto_package()
