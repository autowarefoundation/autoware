cmake_minimum_required(VERSION 3.14)
project(traffic_light_occlusion_predictor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(traffic_light_occlusion_predictor SHARED
  src/nodelet.cpp
  src/occlusion_predictor.cpp
)

rclcpp_components_register_node(traffic_light_occlusion_predictor
  PLUGIN "traffic_light::TrafficLightOcclusionPredictorNodelet"
  EXECUTABLE traffic_light_occlusion_predictor_node
)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(traffic_light_occlusion_predictor ${PCL_LIBRARIES})

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
