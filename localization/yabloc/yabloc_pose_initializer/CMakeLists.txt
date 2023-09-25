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

# ===================================================
# Executable
# Camera
set(TARGET camera_pose_initializer_node)
ament_auto_add_executable(${TARGET}
  src/camera/lane_image.cpp
  src/camera/marker_module.cpp
  src/camera/projector_module.cpp
  src/camera/semantic_segmentation.cpp
  src/camera/camera_pose_initializer_core.cpp
  src/camera/camera_pose_initializer_node.cpp)
target_include_directories(${TARGET} PUBLIC include)
target_include_directories(${TARGET} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${TARGET} ${PCL_LIBRARIES} Sophus::Sophus)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config launch)
