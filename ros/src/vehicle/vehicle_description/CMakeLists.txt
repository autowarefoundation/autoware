cmake_minimum_required(VERSION 2.8.3)
project(vehicle_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch mesh urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)