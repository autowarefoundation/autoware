cmake_minimum_required(VERSION 3.14)
project(autoware_test_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_test_utils SHARED
  src/autoware_test_utils.cpp)

ament_auto_add_library(mock_data_parser SHARED
  src/mock_data_parser.cpp)

target_link_libraries(mock_data_parser
  yaml-cpp
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_autoware_test_utils
    test/test_mock_data_parser.cpp
    test/test_autoware_test_manager.cpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  test_map
  test_data
)
