cmake_minimum_required(VERSION 3.5)
project(yabloc_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# https://github.com/strasdat/Sophus/issues/388
# Sophus's macro cause compile error for the "-Wpedantic" flag, so make sure to import as a system.
# We must always call the following command after "ament_auto_find_build_dependencies()"
# because it rewrite CMAKE_NO_SYSTEM_FROM_IMPORTED to TRUE.
set(CMAKE_NO_SYSTEM_FROM_IMPORTED FALSE)

# ===================================================
# GeographicLib
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)

# ===================================================
# Libraries
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pub_sub.cpp
  src/camera_info_subscriber.cpp
  src/cv_decompress.cpp
  src/pose_conversions.cpp
  src/static_tf_subscriber.cpp
  src/extract_line_segments.cpp
  src/transform_line_segments.cpp
  src/color.cpp
  src/ground_server/ground_server_core.cpp
  src/ground_server/polygon_operation.cpp
  src/ll2_decomposer/ll2_decomposer_core.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC include
)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} Geographic ${PCL_LIBRARIES} Sophus::Sophus)

# ===================================================
# Executables
# ground_server
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yabloc::ground_server::GroundServer"
  EXECUTABLE yabloc_ground_server_node
  EXECUTOR SingleThreadedExecutor
)

# ll2_decomposer
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yabloc::ll2_decomposer::Ll2Decomposer"
  EXECUTABLE yabloc_ll2_decomposer_node
  EXECUTOR SingleThreadedExecutor
)

# ===================================================
ament_export_dependencies(PCL Sophus)

ament_auto_package(INSTALL_TO_SHARE config launch)
