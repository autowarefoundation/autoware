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
<<<<<<< HEAD
include_directories(include 
=======
include_directories(include
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
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
<<<<<<< HEAD
list(APPEND Driveworks_LIBRARIES 
	${CUDA_LIBRARIES} 
	${CUDA_cublas_LIBRARY}
)

list(APPEND Driveworks_LIBRARIES 
	${vibrante_LIBRARIES} 
	${vibrante_Xlibs_LIBRARIES} 
 	nvmedia 
	${EGL_LIBRARIES} 
=======
list(APPEND Driveworks_LIBRARIES
	${CUDA_LIBRARIES}
	${CUDA_cublas_LIBRARY}
)

list(APPEND Driveworks_LIBRARIES
	${vibrante_LIBRARIES}
	${vibrante_Xlibs_LIBRARIES}
 	nvmedia
	${EGL_LIBRARIES}
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
	${CMAKE_THREAD_LIBS_INIT})

set(LIBRARIES ${Driveworks_LIBRARIES})



####################
##  EXECUTABLE    ##
####################
set(NODE_NAME ${PROJECT_NAME}_node)
<<<<<<< HEAD
cuda_add_executable(${NODE_NAME} 
	src/gmsl_camera_multiple/gmsl_node.cpp 
	src/gmsl_camera_multiple/cv_connection.cpp
	src/gmsl_camera_multiple/SekonixGmslCamera.cpp 
=======
cuda_add_executable(${NODE_NAME}
	src/gmsl_camera_multiple/gmsl_node.cpp
	src/gmsl_camera_multiple/cv_connection.cpp
	src/gmsl_camera_multiple/SekonixGmslCamera.cpp
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
	src/gmsl_camera_multiple/DriveWorksApi.cpp
	src/gmsl_camera_multiple/DeviceArguments.cpp
)


add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NODE_NAME} PRIVATE
<<<<<<< HEAD
	${catkin_LIBRARIES} 
=======
	${catkin_LIBRARIES}
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
	driveworks
	${CUDA_LIBRARY}
	${LIBRARIES}
	${GLEW_LIBRARY}
	${BOOST_LIBRARY}
	${OpenCV_LIBS}
<<<<<<< HEAD
   
=======

>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
)


# nodelet
set(NODELET_NAME nodelet_${PROJECT_NAME})
<<<<<<< HEAD
add_library(${NODELET_NAME} 
	src/gmsl_camera_multiple/gmsl_nodelets.cpp 
	src/gmsl_camera_multiple/cv_connection.cpp
	src/gmsl_camera_multiple/SekonixGmslCamera.cpp 
	src/gmsl_camera_multiple/DriveWorksApi.cpp
	src/gmsl_camera_multiple/DeviceArguments.cpp
)
target_link_libraries(${NODELET_NAME} 
    ${catkin_LIBRARIES} 
=======
add_library(${NODELET_NAME}
	src/gmsl_camera_multiple/gmsl_nodelets.cpp
	src/gmsl_camera_multiple/cv_connection.cpp
	src/gmsl_camera_multiple/SekonixGmslCamera.cpp
	src/gmsl_camera_multiple/DriveWorksApi.cpp
	src/gmsl_camera_multiple/DeviceArguments.cpp
)
target_link_libraries(${NODELET_NAME}
    ${catkin_LIBRARIES}
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
    driveworks
    ${CUDA_LIBRARY}
    ${LIBRARIES}
    ${GLEW_LIBRARY}
    ${BOOST_LIBRARY}
<<<<<<< HEAD
    ${OpenCV_LIBS}  
=======
    ${OpenCV_LIBS}
>>>>>>> 78274e28ff4ac39185e5dfcad5d5ce2ba8d12b66
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

