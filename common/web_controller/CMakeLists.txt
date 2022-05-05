cmake_minimum_required(VERSION 3.5)
project(web_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

install(DIRECTORY
  launch
  www
  DESTINATION share/${PROJECT_NAME}/
)

ament_package(
)
