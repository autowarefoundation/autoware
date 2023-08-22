cmake_minimum_required(VERSION 3.14)
project(ground_segmentation)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP)

include_directories(
  include
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GRID_MAP_INCLUDE_DIR}
)

ament_auto_add_library(ground_segmentation SHARED
  src/ray_ground_filter_nodelet.cpp
  src/ransac_ground_filter_nodelet.cpp
  src/scan_ground_filter_nodelet.cpp
)

target_link_libraries(ground_segmentation
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(OPENMP_FOUND)
  set_target_properties(ground_segmentation PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

# ========== Ground Filter ==========
# -- Ray Ground Filter --
rclcpp_components_register_node(ground_segmentation
  PLUGIN "ground_segmentation::RayGroundFilterComponent"
  EXECUTABLE ray_ground_filter_node)

# -- RANSAC Ground Filter --
rclcpp_components_register_node(ground_segmentation
  PLUGIN "ground_segmentation::RANSACGroundFilterComponent"
  EXECUTABLE ransac_ground_filter_node)

# -- Scan Ground Filter --
rclcpp_components_register_node(ground_segmentation
  PLUGIN "ground_segmentation::ScanGroundFilterComponent"
  EXECUTABLE scan_ground_filter_node)


ament_auto_package(INSTALL_TO_SHARE
  launch
  test/data
  config
)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
            NAMES YAML_CPP
            PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})
# cspell:ignore DHAVE, YAMLCPP
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()


if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(test_ray_ground_filter
    test/test_ray_ground_filter.cpp
  )

  target_link_libraries(test_ray_ground_filter
    ground_segmentation
  )

  ament_add_ros_isolated_gtest(test_scan_ground_filter
    test/test_scan_ground_filter.cpp
  )

  target_link_libraries(test_scan_ground_filter
    ground_segmentation
    ${YAML_CPP_LIBRARIES}
  )
endif()
