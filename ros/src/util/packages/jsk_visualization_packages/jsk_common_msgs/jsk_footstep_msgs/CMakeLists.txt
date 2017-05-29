# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_footstep_msgs)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS geometry_msgs actionlib_msgs message_generation)

add_message_files(
  DIRECTORY msg
  FILES Footstep.msg FootstepArray.msg
)

#add_service_files(
#  FILES
#)

add_action_files(
  DIRECTORY action
  FILES PlanFootsteps.action ExecFootsteps.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs actionlib_msgs
)

# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS
    CATKIN_DEPENDS  geometry_msgs actionlib_msgs message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

