cmake_minimum_required(VERSION 3.14)
project(route_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(route_handler SHARED
  src/route_handler.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_route_handler
    test/test_route_handler.cpp)

  target_link_libraries(test_route_handler
    route_handler
  )

endif()

ament_auto_package(INSTALL_TO_SHARE
  test_route
)
