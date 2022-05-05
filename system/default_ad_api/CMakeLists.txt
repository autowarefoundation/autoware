cmake_minimum_required(VERSION 3.8)
project(default_ad_api)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/interface_version.cpp
)

# workaround to allow deprecated function to build on both galactic and rolling
if(${rosidl_generator_cpp_VERSION} VERSION_LESS 2.5.0)
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_DEPRECATED_TO_YAML
  )
endif()

rclcpp_components_register_nodes(${PROJECT_NAME} "default_ad_api::InterfaceVersionNode")

if(BUILD_TESTING)
  add_launch_test(test/main.test.py)
endif()

install(
  PROGRAMS script/web_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch test)
