cmake_minimum_required(VERSION 3.14)
project(traffic_light_visualization)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)

ament_auto_add_library(traffic_light_roi_visualizer_nodelet SHARED
  src/traffic_light_roi_visualizer/nodelet.cpp
)

target_link_libraries(traffic_light_roi_visualizer_nodelet
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(traffic_light_roi_visualizer_nodelet
  PLUGIN "traffic_light::TrafficLightRoiVisualizerNodelet"
  EXECUTABLE traffic_light_visualization_node
)

ament_auto_add_executable(traffic_light_map_visualizer_node
  src/traffic_light_map_visualizer/node.cpp
  src/traffic_light_map_visualizer/main.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
