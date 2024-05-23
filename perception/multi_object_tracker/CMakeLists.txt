cmake_minimum_required(VERSION 3.14)
project(multi_object_tracker)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

# Generate exe file
set(MULTI_OBJECT_TRACKER_SRC
  src/multi_object_tracker_core.cpp
  src/debugger/debugger.cpp
  src/debugger/debug_object.cpp
  src/processor/processor.cpp
  src/processor/input_manager.cpp
  src/data_association/data_association.cpp
  src/data_association/mu_successive_shortest_path/mu_successive_shortest_path_wrapper.cpp
  src/tracker/motion_model/motion_model_base.cpp
  src/tracker/motion_model/bicycle_motion_model.cpp
  # cspell: ignore ctrv
  src/tracker/motion_model/ctrv_motion_model.cpp
  src/tracker/motion_model/cv_motion_model.cpp
  src/tracker/model/tracker_base.cpp
  src/tracker/model/big_vehicle_tracker.cpp
  src/tracker/model/normal_vehicle_tracker.cpp
  src/tracker/model/multiple_vehicle_tracker.cpp
  src/tracker/model/bicycle_tracker.cpp
  src/tracker/model/pedestrian_tracker.cpp
  src/tracker/model/pedestrian_and_bicycle_tracker.cpp
  src/tracker/model/unknown_tracker.cpp
  src/tracker/model/pass_through_tracker.cpp
)

ament_auto_add_library(multi_object_tracker_node SHARED
  ${MULTI_OBJECT_TRACKER_SRC}
)

target_link_libraries(multi_object_tracker_node
  Eigen3::Eigen
  glog::glog
)

ament_auto_add_executable(${PROJECT_NAME}
  src/multi_object_tracker_node.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
