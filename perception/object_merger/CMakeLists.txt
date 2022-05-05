cmake_minimum_required(VERSION 3.14)
project(object_merger)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common filters)

ament_auto_add_library(object_association_merger SHARED
  src/object_association_merger/utils/utils.cpp
  src/object_association_merger/data_association/data_association.cpp
  src/object_association_merger/data_association/successive_shortest_path/successive_shortest_path.cpp
  src/object_association_merger/node.cpp
)

target_link_libraries(object_association_merger ${PCL_LIBRARIES})

rclcpp_components_register_node(object_association_merger
  PLUGIN "object_association::ObjectAssociationMergerNode"
  EXECUTABLE object_association_merger_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
