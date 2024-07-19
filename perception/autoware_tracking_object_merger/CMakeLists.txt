cmake_minimum_required(VERSION 3.8)
project(autoware_tracking_object_merger VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wconversion)
endif()

find_package(autoware_cmake REQUIRED)
find_package(glog REQUIRED)
autoware_package()


# find dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/association/data_association.cpp
  src/association/solver/mu_successive_shortest_path_wrapper.cpp
  src/decorative_tracker_merger_node.cpp
  src/utils/utils.cpp
  src/utils/tracker_state.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  glog::glog
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::tracking_object_merger::DecorativeTrackerMergerNode"
  EXECUTABLE decorative_tracker_merger_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
