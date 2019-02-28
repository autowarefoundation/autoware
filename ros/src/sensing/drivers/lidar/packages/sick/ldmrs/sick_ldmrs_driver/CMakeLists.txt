cmake_minimum_required(VERSION 2.8.3)
project(sick_ldmrs_driver)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  pcl_conversions
  roscpp
  roslaunch
  sensor_msgs
  sick_ldmrs_msgs
  tf
)

add_subdirectory(src/driver)

find_package(Boost REQUIRED COMPONENTS system)

# The following is commented because we're importing sick_ldmrs from src/dirver
# find_package(SickLDMRS REQUIRED)

generate_dynamic_reconfigure_options(cfg/SickLDMRSDriver.cfg)

catkin_package(
    DEPENDS Boost
    CATKIN_DEPENDS roscpp sensor_msgs diagnostic_updater dynamic_reconfigure pcl_conversions sick_ldmrs_msgs
#    LIBRARIES 
    INCLUDE_DIRS include
    )

include_directories(include 
	src/driver/src
	${catkin_INCLUDE_DIRS}
	)

add_executable(sick_ldmrs_node
    src/sick_ldmrs_node.cpp)

# sick_ldmrs is an imported target from libsick_ldmrs (in src/driver), we'd use
# ${SICK_LDMRS_LIBRARIES} as defined by find_package(SickLDMRS) otherwise
target_link_libraries(sick_ldmrs_node
    ${catkin_LIBRARIES}
    sick_ldmrs)
add_dependencies(sick_ldmrs_node ${PROJECT_NAME}_gencfg)
add_dependencies(sick_ldmrs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS sick_ldmrs_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
