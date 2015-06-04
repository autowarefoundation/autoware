cmake_minimum_required(VERSION 2.8.3)
project(vlg22c_cam)

find_package(catkin REQUIRED COMPONENTS 
	image_transport 
	roscpp 
	std_msgs 
	sensor_msgs 
	camera_info_manager)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
#pkg_check_modules(bgapi libbgapi REQUIRED)



###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES ${PROJECT_NAME}
)
               
                       
add_definitions(-D_GNULINUX)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  BaumerGAPI/Components/Dev/C_C++/Inc
  nodes/vlg22/src/
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  nodes/vlg22/src/vlg22_main.cpp
  nodes/vlg22/src/bgapi_init.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  bgapi
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation

install(DIRECTORY BaumerGAPI/Components/Bin/Systems/GigE/
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   FILES_MATCHING PATTERN "*.so.*" PATTERN "*.so"
)


install(TARGETS ${PROJECT_NAME}_node 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
