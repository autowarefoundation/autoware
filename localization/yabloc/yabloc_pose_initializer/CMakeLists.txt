cmake_minimum_required(VERSION 3.5)
project(yabloc_pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common kdtree)

# Sophus
find_package(Sophus REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# ===================================================
# Executable
# Camera
ament_auto_add_library(${PROJECT_NAME}
  src/camera/lane_image.cpp
  src/camera/marker_module.cpp
  src/camera/projector_module.cpp
  src/camera/semantic_segmentation.cpp
  src/camera/camera_pose_initializer_core.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Sophus::Sophus)
ament_target_dependencies(${PROJECT_NAME} OpenCV)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yabloc::CameraPoseInitializer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config launch)
