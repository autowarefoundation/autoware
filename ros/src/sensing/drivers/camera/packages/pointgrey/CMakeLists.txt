cmake_minimum_required(VERSION 2.8.12)

project(autoware_pointgrey_drivers)

find_package(catkin REQUIRED COMPONENTS
	autoware_build_flags
	roscpp
	std_msgs
	message_generation
	tf
	sensor_msgs
    camera_info_manager
    image_transport
    cv_bridge
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

## pkg-config libraries
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
    LIBRARIES ${PROJECT_NAME}
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
find_package(Spinnaker QUIET)
if(NOT Spinnaker_FOUND)
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(Spinnaker_LIBRARIES Spinnaker_INCLUDE_DIRS)
endif()

include_directories(include
                    ${Spinnaker_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)


link_directories("${Spinnaker_LIBRARIES}")

add_executable(spinnaker_camera
    nodes/spinnaker/spinnaker.cpp
)

target_link_libraries(spinnaker_camera
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${Spinnaker_LIBRARIES}
)
