cmake_minimum_required(VERSION 3.5)

project(autoware_auto_cmake NONE)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "autoware_auto_cmake-extras.cmake"
)

if(BUILD_TESTING)
    ament_lint_cmake(${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_auto_package(INSTALL_TO_SHARE
  cmake
)
