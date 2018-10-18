cmake_minimum_required(VERSION 2.8.3)
project(gmsl_camera)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error -Wno-deprecated-declarations -std=c++11")

##################
## ROS packages ##
##################
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	#autoware_build_flags
	roscpp
	sensor_msgs
	std_msgs
	cv_bridge
	image_transport
	camera_info_manager
	nodelet
)


##############################
## Set build configurations ##
##############################
include(FindDriveWorksSDK.cmake)
link_directories(${Driveworks_LIBRARY})
set(VIBRANTE FALSE)
add_definitions(-DVIBRANTE)

#####################
## System packages ##
#####################
find_package(CUDA)
# Need CUDA to run this package
if(CUDA_FOUND)
find_package(Threads REQUIRED)
find_package(TensorRT REQUIRED)
find_package(CuDNN REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	LIBRARIES ${PROJECT_NAME}
)

#########################
##  Include directory  ##
#########################
include_directories(include
	${catkin_INCLUDE_DIRS}
	${Driveworks_INCLUDE_DIR}
	${CUDA_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/src
	${NVMEDIA_DIRS}
	${BOOST_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
)

####################
## Driveworks SDK ##
####################
list(APPEND Driveworks_LIBRARIES
	${CUDA_LIBRARIES}
	${CUDA_cublas_LIBRARY}
)

list(APPEND Driveworks_LIBRARIES
	${vibrante_LIBRARIES}
	${vibrante_Xlibs_LIBRARIES}
 	nvmedia
	${EGL_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT})

set(LIBRARIES ${Driveworks_LIBRARIES})



####################
##  EXECUTABLE    ##
####################
set(NODE_NAME ${PROJECT_NAME}_node)
cuda_add_executable(${NODE_NAME}
	src/gmsl_camera_multiple/gmsl_node.cpp
	src/gmsl_camera_multiple/cv_connection.cpp
	src/gmsl_camera_multiple/SekonixGmslCamera.cpp
	src/gmsl_camera_multiple/DriveWorksApi.cpp
	src/gmsl_camera_multiple/DeviceArguments.cpp
)


add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE_NAME} PRIVATE
	${catkin_LIBRARIES}
	driveworks
	${CUDA_LIBRARY}
	${LIBRARIES}
	${GLEW_LIBRARY}
	${BOOST_LIBRARY}
	${OpenCV_LIBS}

)


# nodelet
set(NODELET_NAME nodelet_${PROJECT_NAME})
add_library(${NODELET_NAME}
	src/gmsl_camera_multiple/gmsl_nodelets.cpp
	src/gmsl_camera_multiple/cv_connection.cpp
	src/gmsl_camera_multiple/SekonixGmslCamera.cpp
	src/gmsl_camera_multiple/DriveWorksApi.cpp
	src/gmsl_camera_multiple/DeviceArguments.cpp
)
target_link_libraries(${NODELET_NAME}
    ${catkin_LIBRARIES}
    driveworks
    ${CUDA_LIBRARY}
    ${LIBRARIES}
    ${GLEW_LIBRARY}
    ${BOOST_LIBRARY}
    ${OpenCV_LIBS}
)


##################
##   Install    ##
##################
install(TARGETS ${NODE_NAME} ${NODELET_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
	nodelet_gmsl_camera.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

else()
	message("gmsl_camera driver will not be built. CUDA was not found.")
endif()

