cmake_minimum_required(VERSION 3.14)
project(autoware_pose_instability_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_instability_detector.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose_instability_detector::PoseInstabilityDetector"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test.cpp
    src/pose_instability_detector.cpp
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
