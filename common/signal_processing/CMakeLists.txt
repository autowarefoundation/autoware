cmake_minimum_required(VERSION 3.14)
project(signal_processing)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(lowpass_filter_1d SHARED
  src/lowpass_filter_1d.cpp
)

if(BUILD_TESTING)
  ament_add_gtest(test_signal_processing
    test/src/lowpass_filter_1d_test.cpp
  )
  target_link_libraries(test_signal_processing
    lowpass_filter_1d
  )
endif()

ament_auto_package()
