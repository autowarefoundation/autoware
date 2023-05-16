cmake_minimum_required(VERSION 3.5)
project(sampler_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

ament_auto_add_library(sampler_common SHARED
  DIRECTORY src/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_sampler_common
    test/test_transform.cpp
    test/test_structures.cpp
  )

  target_link_libraries(test_sampler_common
    sampler_common
  )
endif()

ament_auto_package()
