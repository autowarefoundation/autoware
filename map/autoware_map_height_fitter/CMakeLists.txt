cmake_minimum_required(VERSION 3.14)
project(autoware_map_height_fitter)

find_package(autoware_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map_height_fitter.cpp
  src/map_height_fitter_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# When adding `<depend>autoware_lanelet2_extension</depend>` to package.xml, many warnings are generated.
# These are treated as errors in compile, so pedantic warnings are disabled for this package.
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-pedantic)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::map_height_fitter::MapHeightFitterNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
