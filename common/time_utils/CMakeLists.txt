cmake_minimum_required(VERSION 3.6)

project(time_utils)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED src/time_utils/time_utils.cpp)
autoware_set_compile_options(${PROJECT_NAME})

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # TODO(c.ho) unit tests
endif()

ament_auto_package()
