cmake_minimum_required(VERSION 3.14)
project(autoware_auto_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  set(TEST_COMMON test_common_gtest)
  ament_add_ros_isolated_gtest(${TEST_COMMON}
    test/gtest_main.cpp
    test/test_bool_comparisons.cpp
    test/test_byte_reader.cpp
    test/test_float_comparisons.cpp
    test/test_mahalanobis_distance.cpp
    test/test_message_field_adapters.cpp
    test/test_template_utils.cpp
    test/test_angle_utils.cpp
    test/test_type_name.cpp
    test/test_type_traits.cpp
  )
  target_compile_options(${TEST_COMMON} PRIVATE -Wno-sign-conversion)
  target_include_directories(${TEST_COMMON} PRIVATE include)
  ament_target_dependencies(${TEST_COMMON}
    builtin_interfaces
    Eigen3
    geometry_msgs
  )
endif()

ament_auto_package()
