cmake_minimum_required(VERSION 3.5)
project(autoware_state_monitor)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Target
## Set source files
set(AUTOWARE_STATE_MONITOR_SRC
  src/autoware_state_monitor_node/autoware_state_monitor_node.cpp
  src/autoware_state_monitor_node/state_machine.cpp
  src/autoware_state_monitor_node/diagnostics.cpp
)

## Add executables
ament_auto_add_executable(autoware_state_monitor
  src/autoware_state_monitor_node/main.cpp
  ${AUTOWARE_STATE_MONITOR_SRC}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
