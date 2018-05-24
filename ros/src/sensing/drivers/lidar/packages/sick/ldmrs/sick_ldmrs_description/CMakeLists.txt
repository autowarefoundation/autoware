cmake_minimum_required(VERSION 2.8.3)
project(sick_ldmrs_description)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
