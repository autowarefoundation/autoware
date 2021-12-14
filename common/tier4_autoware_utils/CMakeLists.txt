cmake_minimum_required(VERSION 3.5)
project(tier4_autoware_utils)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)

ament_auto_add_library(tier4_autoware_utils SHARED
  src/tier4_autoware_utils.cpp
  src/planning/planning_marker_helper.cpp
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_gtest(test_tier4_autoware_utils ${test_files})

  target_link_libraries(test_tier4_autoware_utils
    tier4_autoware_utils
  )
endif()

ament_auto_package()
