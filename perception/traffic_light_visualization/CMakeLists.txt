cmake_minimum_required(VERSION 3.5)
project(traffic_light_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

###########
## Build ##
###########

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_add_executable(traffic_light_map_visualizer_node
  src/traffic_light_map_visualizer/node.cpp
  src/traffic_light_map_visualizer/main.cpp
)

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
