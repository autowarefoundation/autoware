cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_visualization)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)

ament_auto_add_library(traffic_light_roi_visualizer SHARED
  src/traffic_light_roi_visualizer/node.cpp
  src/traffic_light_roi_visualizer/shape_draw.cpp
)

target_link_libraries(traffic_light_roi_visualizer
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(traffic_light_roi_visualizer
  PLUGIN "autoware::traffic_light::TrafficLightRoiVisualizerNode"
  EXECUTABLE traffic_light_visualization_node
)

ament_auto_add_library(traffic_light_map_visualizer SHARED
  src/traffic_light_map_visualizer/node.cpp
)

rclcpp_components_register_node(traffic_light_map_visualizer
  PLUGIN "autoware::traffic_light::TrafficLightMapVisualizerNode"
  EXECUTABLE traffic_light_map_visualizer_node
)

# Copy the assets directory to the installation directory
install(
    DIRECTORY images/
    DESTINATION share/${PROJECT_NAME}/images
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
