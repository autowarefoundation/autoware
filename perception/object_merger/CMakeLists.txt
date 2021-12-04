cmake_minimum_required(VERSION 3.5)
project(object_merger)

find_package(PCL REQUIRED COMPONENTS common filters)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
)
