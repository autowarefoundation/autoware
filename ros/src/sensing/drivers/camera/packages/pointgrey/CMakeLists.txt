cmake_minimum_required(VERSION 2.8.12)

project(autoware_pointgrey_drivers)

find_package(catkin REQUIRED COMPONENTS
	autoware_build_flags
	roscpp
	std_msgs
	message_generation
	tf
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########

if(EXISTS "/usr/include/flycapture")
	include_directories(
		/usr/include/flycapture
		${catkin_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS}
	)
	add_executable(grasshopper3_camera
		nodes/grasshopper3/grasshopper3.cpp
	)

	target_link_libraries(grasshopper3_camera
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
		flycapture
	)
else()
	message("'FlyCapture SDK' is not installed. 'grasshopper3_camera' will not be built.")
endif()


if(EXISTS "/usr/include/ladybug")

	if(${CMAKE_VERSION} VERSION_LESS "3.1.3")
		include(FindXercesC.cmake)
	else()
		find_package(XercesC REQUIRED)
	endif()

	include_directories(
		/usr/include/ladybug
		nodes/ladybug
		${catkin_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${XercesC_INCLUDE_DIRS}
	)

	add_executable(ladybug_camera
		nodes/ladybug/ladybug.cpp
	)

	target_link_libraries(ladybug_camera
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
		${XercesC_LIBRARIES}
		flycapture
		ladybug
	)
else()
	message("'SDK for Ladybug' is not installed. 'ladybug_camera' will not be built.")
endif()
