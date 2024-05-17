cmake_minimum_required(VERSION 3.14)
project(reaction_analyzer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(reaction_analyzer SHARED
  include/reaction_analyzer_node.hpp
  src/reaction_analyzer_node.cpp
  include/subscriber.hpp
  src/subscriber.cpp
  include/topic_publisher.hpp
  src/topic_publisher.cpp
  include/utils.hpp
  src/utils.cpp
)

target_include_directories(reaction_analyzer
  SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(reaction_analyzer
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(reaction_analyzer
  PLUGIN "reaction_analyzer::ReactionAnalyzerNode"
  EXECUTABLE reaction_analyzer_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
  param
  launch
)
