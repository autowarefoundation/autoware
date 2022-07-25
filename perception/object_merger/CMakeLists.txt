cmake_minimum_required(VERSION 3.14)
project(object_merger)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(mu_successive_shortest_path SHARED
  src/object_association_merger/data_association/mu_successive_shortest_path/mu_successive_shortest_path_wrapper.cpp
)

ament_auto_add_library(object_association_merger SHARED
  src/object_association_merger/data_association/data_association.cpp
  src/object_association_merger/node.cpp
)

target_link_libraries(object_association_merger
  mu_successive_shortest_path
  Eigen3::Eigen
)

rclcpp_components_register_node(object_association_merger
  PLUGIN "object_association::ObjectAssociationMergerNode"
  EXECUTABLE object_association_merger_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
