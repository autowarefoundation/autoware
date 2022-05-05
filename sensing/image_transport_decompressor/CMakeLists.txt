cmake_minimum_required(VERSION 3.14)
project(image_transport_decompressor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(image_transport_decompressor SHARED
  src/image_transport_decompressor.cpp
)

target_link_libraries(image_transport_decompressor
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(image_transport_decompressor
  PLUGIN "image_preprocessor::ImageTransportDecompressor"
  EXECUTABLE image_transport_decompressor_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
