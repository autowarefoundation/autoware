cmake_minimum_required(VERSION 3.5)
project(yabloc_image_processing)

find_package(autoware_cmake REQUIRED)
autoware_package()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/line_segment_detector/line_segment_detector_core.cpp
  src/graph_segment/graph_segment_core.cpp
  src/graph_segment/similar_area_searcher.cpp
  src/segment_filter/segment_filter_core.cpp
  src/undistort/undistort_node.cpp
  src/line_segments_overlay/line_segments_overlay_core.cpp
  src/lanelet2_overlay/lanelet2_overlay_core.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# ===================================================
# Executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yabloc::graph_segment::GraphSegment"
  EXECUTABLE yabloc_graph_segment_node
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yabloc::lanelet2_overlay::Lanelet2Overlay"
  EXECUTABLE yabloc_lanelet2_overlay_node
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yabloc::line_segment_detector::LineSegmentDetector"
  EXECUTABLE yabloc_line_segment_detector_node
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yabloc::line_segments_overlay::LineSegmentsOverlay"
  EXECUTABLE yabloc_line_segments_overlay_node
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yabloc::segment_filter::SegmentFilter"
  EXECUTABLE yabloc_segment_filter_node
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "yabloc::undistort::UndistortNode"
  EXECUTABLE yabloc_undistort_node
  EXECUTOR SingleThreadedExecutor
)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE config launch)
