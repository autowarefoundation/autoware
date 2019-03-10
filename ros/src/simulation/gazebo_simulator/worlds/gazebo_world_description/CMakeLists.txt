cmake_minimum_required(VERSION 2.8.3)
project(gazebo_world_description)

find_package(catkin REQUIRED)

catkin_package()
add_custom_target(integrate_model ALL /bin/bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/integrate_model_files.sh)
add_custom_target(integrate_world ALL /bin/bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/integrate_world_files.sh)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
