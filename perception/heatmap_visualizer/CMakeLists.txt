cmake_minimum_required(VERSION 3.14)
project(heatmap_visualizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(heatmap_visualizer_component SHARED
    src/heatmap_visualizer_node.cpp
    src/utils.cpp
)

rclcpp_components_register_node(heatmap_visualizer_component
    PLUGIN "heatmap_visualizer::HeatmapVisualizerNode"
    EXECUTABLE heatmap_visualizer
)

ament_auto_package(INSTALL_TO_SHARE
    launch
)
