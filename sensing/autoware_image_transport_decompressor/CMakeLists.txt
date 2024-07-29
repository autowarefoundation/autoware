cmake_minimum_required(VERSION 3.14)
project(autoware_image_transport_decompressor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/image_transport_decompressor.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::image_preprocessor::ImageTransportDecompressor"
  EXECUTABLE image_transport_decompressor_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
