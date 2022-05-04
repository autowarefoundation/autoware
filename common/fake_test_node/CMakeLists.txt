cmake_minimum_required(VERSION 3.5)
project(fake_test_node)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(fake_test_node SHARED src/fake_test_node.cpp)
autoware_set_compile_options(fake_test_node)

## Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fake_test_node
    test/test_fake_test_node.cpp
  )
  autoware_set_compile_options(test_fake_test_node)
  add_dependencies(test_fake_test_node fake_test_node)
  target_link_libraries(test_fake_test_node fake_test_node)
endif()

ament_auto_package()
