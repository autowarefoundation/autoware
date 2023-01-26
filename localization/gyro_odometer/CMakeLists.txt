cmake_minimum_required(VERSION 3.14)
project(gyro_odometer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/gyro_odometer_node.cpp
  src/gyro_odometer_core.cpp
)

target_link_libraries(${PROJECT_NAME} fmt)

ament_auto_add_library(gyro_odometer_node SHARED
  src/gyro_odometer_core.cpp
)

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
    gyro_odometer_node
  )
endif()


ament_auto_package(INSTALL_TO_SHARE
  launch
)
