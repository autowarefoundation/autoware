cmake_minimum_required(VERSION 3.14)
project(autoware_gyro_odometer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gyro_odometer_core.cpp
)

target_link_libraries(${PROJECT_NAME} fmt)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_gyro_odometer
    test/test_main.cpp
    test/test_gyro_odometer_pubsub.cpp
    test/test_gyro_odometer_helper.cpp
  )
  ament_target_dependencies(test_gyro_odometer
    rclcpp
  )
  target_link_libraries(test_gyro_odometer
    ${PROJECT_NAME}
  )
  target_include_directories(test_gyro_odometer PRIVATE
    src
  )
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::gyro_odometer::GyroOdometerNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
