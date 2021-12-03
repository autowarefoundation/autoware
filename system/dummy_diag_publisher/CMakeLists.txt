cmake_minimum_required(VERSION 3.5)
project(dummy_diag_publisher)

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

### Target executable
set(DUMMY_DIAG_PUBLISHER_SRC
  src/dummy_diag_publisher_node/dummy_diag_publisher_node.cpp)

ament_auto_add_executable(${PROJECT_NAME}
  src/dummy_diag_publisher_node/main.cpp
  ${DUMMY_DIAG_PUBLISHER_SRC}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
