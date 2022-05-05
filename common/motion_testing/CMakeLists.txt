cmake_minimum_required(VERSION 3.14)
project(motion_testing)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED src/motion_testing/motion_testing.cpp)

if(BUILD_TESTING)
  ament_add_gtest(motion_testing_unit_tests
    test/gtest_main.cpp
    test/constant_trajectory.cpp
    test/trajectory_checks.cpp)
  target_compile_options(motion_testing_unit_tests PRIVATE -Wno-sign-conversion)
  target_link_libraries(motion_testing_unit_tests ${PROJECT_NAME})
  add_dependencies(motion_testing_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
