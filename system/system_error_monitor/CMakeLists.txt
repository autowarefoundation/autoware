cmake_minimum_required(VERSION 3.5)
project(system_error_monitor)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Target
## system_error_monitor_node
set(SYSTEM_ERROR_MONITOR_SRC
  src/system_error_monitor_core.cpp
)

ament_auto_add_executable(${PROJECT_NAME}
  src/system_error_monitor_node.cpp
  ${SYSTEM_ERROR_MONITOR_SRC}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
## executables and libraries
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
