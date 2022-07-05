cmake_minimum_required(VERSION 3.14)
project(costmap_generator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common io)
find_package(FLANN REQUIRED)

include_directories(
  include
  SYSTEM
  ${GRID_MAP_INCLUDE_DIR}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(costmap_generator_lib SHARED
  nodes/costmap_generator/points_to_costmap.cpp
  nodes/costmap_generator/objects_to_costmap.cpp
  nodes/costmap_generator/object_map_utils.cpp
)
target_link_libraries(costmap_generator_lib
  ${PCL_LIBRARIES}
  FLANN::FLANN
)

if(${PCL_VERSION} GREATER_EQUAL 1.12.1)
  find_package(Qhull REQUIRED)
  target_link_libraries(costmap_generator_lib
    QHULL::QHULL
  )
endif()

ament_auto_add_library(costmap_generator_node SHARED
  nodes/costmap_generator/costmap_generator_node.cpp
)
target_link_libraries(costmap_generator_node
  ${PCL_LIBRARIES}
  costmap_generator_lib
)

rclcpp_components_register_node(costmap_generator_node
  PLUGIN "CostmapGenerator"
  EXECUTABLE costmap_generator
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_points_to_costmap
    test/test_points_to_costmap.cpp
  )

  target_link_libraries(test_points_to_costmap
    costmap_generator_lib
  )

  ament_add_ros_isolated_gtest(test_objects_to_costmap
    test/test_objects_to_costmap.cpp
  )

  target_link_libraries(test_objects_to_costmap
    costmap_generator_lib
  )
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
