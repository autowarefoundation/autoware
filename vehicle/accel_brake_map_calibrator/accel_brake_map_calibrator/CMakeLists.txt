cmake_minimum_required(VERSION 3.14)
project(accel_brake_map_calibrator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(accel_brake_map_calibrator
  src/accel_brake_map_calibrator_node.cpp
  src/main.cpp
)
ament_target_dependencies(accel_brake_map_calibrator)

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
