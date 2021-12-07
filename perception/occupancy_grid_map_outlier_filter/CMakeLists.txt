cmake_minimum_required(VERSION 3.5)
project(occupancy_grid_map_outlier_filter)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Ignore PCL errors in Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP)
ament_auto_find_build_dependencies()


###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GRID_MAP_INCLUDE_DIR}
)

ament_auto_add_library(occupancy_grid_map_outlier_filter SHARED
  src/occupancy_grid_map_outlier_filter_nodelet.cpp
)

target_link_libraries(occupancy_grid_map_outlier_filter
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(OPENMP_FOUND)
  set_target_properties(occupancy_grid_map_outlier_filter PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()


# -- Occupancy Grid Map Outlier Filter --
rclcpp_components_register_node(occupancy_grid_map_outlier_filter
  PLUGIN "occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent"
  EXECUTABLE occupancy_grid_map_outlier_filter_node)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE)
