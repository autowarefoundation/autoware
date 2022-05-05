cmake_minimum_required(VERSION 3.6)
project(motion_testing)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED src/motion_testing/motion_testing.cpp)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

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
