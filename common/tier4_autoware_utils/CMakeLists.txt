cmake_minimum_required(VERSION 3.5)
project(tier4_autoware_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_add_library(tier4_autoware_utils SHARED
  src/tier4_autoware_utils.cpp
  src/planning/planning_marker_helper.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(tier4_autoware_utils PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_gtest(test_tier4_autoware_utils ${test_files})

  target_link_libraries(test_tier4_autoware_utils
    tier4_autoware_utils
  )
endif()

ament_auto_package()
