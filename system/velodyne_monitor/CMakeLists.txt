cmake_minimum_required(VERSION 3.14)
project(velodyne_monitor)

find_package(autoware_cmake REQUIRED)
find_package(fmt)
autoware_package()

### Target executable
ament_auto_add_executable(velodyne_monitor
  src/velodyne_monitor_node.cpp
  src/velodyne_monitor.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(velodyne_monitor cpprest crypto fmt)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
