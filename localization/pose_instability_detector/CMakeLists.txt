cmake_minimum_required(VERSION 3.14)
project(pose_instability_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(pose_instability_detector
  src/main.cpp
  src/pose_instability_detector.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_pose_instability_detector
    test/test.cpp
    src/pose_instability_detector.cpp
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
