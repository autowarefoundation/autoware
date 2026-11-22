cmake_minimum_required(VERSION 3.14)
project(autoware_geo_pose_projector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/geo_pose_projector.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::geo_pose_projector::GeoPoseProjector"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_covariance_conversion
    test/test_covariance_conversion.cpp
  )
  target_include_directories(test_covariance_conversion PRIVATE src)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
