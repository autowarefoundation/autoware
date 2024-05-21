cmake_minimum_required(VERSION 3.14)
project(planning_test_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(mock_data_parser SHARED
  src/mock_data_parser.cpp)

target_link_libraries(mock_data_parser
  yaml-cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_mock_data_parser
    test/test_mock_data_parser.cpp)

  target_link_libraries(test_mock_data_parser
    mock_data_parser)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  test_map
  test_data
)
