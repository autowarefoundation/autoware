cmake_minimum_required(VERSION 3.14)
project(lidar_centerpoint_tvm)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(tvm_runtime_DIR ${tvm_vendor_DIR})
find_package(tvm_runtime CONFIG REQUIRED)

# Gather neural network information.
set(${PROJECT_NAME}_BACKEND llvm CACHE STRING "${PROJECT_NAME} neural network backend")
set(MODEL_NAME_ENCODER centerpoint_encoder)

# Get neural network.
set(NN_DEPENDENCY_ENCODER "")
get_neural_network(${MODEL_NAME_ENCODER} ${${PROJECT_NAME}_BACKEND} NN_DEPENDENCY_ENCODER)

set(MODEL_NAME_BACKBONE centerpoint_backbone)

# Get neural network.
set(NN_DEPENDENCY_BACKBONE "")
get_neural_network(${MODEL_NAME_BACKBONE} ${${PROJECT_NAME}_BACKEND} NN_DEPENDENCY_BACKBONE)

if((NOT NN_DEPENDENCY_ENCODER STREQUAL "") AND (NOT NN_DEPENDENCY_BACKBONE STREQUAL ""))
  ## centerpoint_tvm ##
  ament_auto_add_library(${PROJECT_NAME} SHARED
    data/models/${MODEL_NAME_ENCODER}/inference_engine_tvm_config.hpp
    data/models/${MODEL_NAME_BACKBONE}/inference_engine_tvm_config.hpp
    lib/centerpoint_tvm.cpp
    lib/utils.cpp
    lib/ros_utils.cpp
    lib/network/scatter.cpp
    lib/preprocess/pointcloud_densification.cpp
    lib/preprocess/voxel_generator.cpp
    lib/preprocess/generate_features.cpp
    lib/postprocess/circle_nms.cpp
    lib/postprocess/generate_detected_boxes.cpp
  )

  add_dependencies(${PROJECT_NAME} ${NN_DEPENDENCY_ENCODER})
  add_dependencies(${PROJECT_NAME} ${NN_DEPENDENCY_BACKBONE})

  target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-sign-conversion" "-Wno-conversion")

  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    "${tvm_vendor_INCLUDE_DIRS}"
  )

  target_link_libraries(${PROJECT_NAME}
    ${tvm_runtime_LIBRARIES}
  )

  target_include_directories(${PROJECT_NAME} PRIVATE
    data/models
  )

  ## node ##
  ament_auto_add_library(lidar_centerpoint_tvm_component SHARED
    src/node.cpp
  )

  target_link_libraries(lidar_centerpoint_tvm_component
    ${PROJECT_NAME}
  )

  rclcpp_components_register_node(lidar_centerpoint_tvm_component
    PLUGIN "autoware::perception::lidar_centerpoint_tvm::LidarCenterPointTVMNode"
    EXECUTABLE lidar_centerpoint_tvm_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    launch
    config
  )
else()
  message(WARNING "Neural network not found, skipping package.")
  ament_auto_package()
endif()
