cmake_minimum_required(VERSION 3.5)
project(accel_brake_map_calibrator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(accel_brake_map_calibrator
  src/accel_brake_map_calibrator_node.cpp
  src/main.cpp
)
ament_target_dependencies(accel_brake_map_calibrator)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
  scripts/__init__.py
  scripts/config.py
  scripts/delay_estimator.py
  scripts/plotter.py
  scripts/view_statistics.py
  scripts/calc_utils.py
  scripts/csv_reader.py
  scripts/log_analyzer.py
  scripts/view_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  rviz)
