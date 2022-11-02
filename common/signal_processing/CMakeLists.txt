cmake_minimum_required(VERSION 3.14)
project(signal_processing)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(lowpass_filters SHARED
  src/lowpass_filter_1d.cpp
  src/lowpass_filter.cpp
  src/butterworth.cpp)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_signal_processing
    test/src/lowpass_filter_1d_test.cpp
    test/src/lowpass_filter_test.cpp
    test/src/butterworth_filter_test.cpp)

  target_include_directories(test_signal_processing PUBLIC test/include)
  target_link_libraries(test_signal_processing
    lowpass_filters)

endif()

ament_auto_package()
