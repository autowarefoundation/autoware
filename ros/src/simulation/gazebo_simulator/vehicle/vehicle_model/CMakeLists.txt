cmake_minimum_required(VERSION 2.8.3)
project(vehicle_model)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  effort_controllers
  gazebo_ros_control
  position_controllers
  transmission_interface
  velocity_controllers
)

catkin_package(
  CATKIN_DEPENDS    
    effort_controllers
    gazebo_ros_control
    position_controllers
    transmission_interface
    velocity_controllers
)

install(DIRECTORY mesh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
