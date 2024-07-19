cmake_minimum_required(VERSION 3.14)
project(autoware_object_merger)

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

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/association/data_association.cpp
  src/association/solver/mu_successive_shortest_path_wrapper.cpp
  src/object_association_merger_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::object_merger::ObjectAssociationMergerNode"
  EXECUTABLE object_association_merger_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
