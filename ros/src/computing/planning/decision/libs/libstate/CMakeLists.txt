cmake_minimum_required(VERSION 2.8.3)
project(state)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	autoware_msgs
	euc
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES state
	CATKIN_DEPENDS roscpp autoware_msgs
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)

set(TARGET state)
set(TARGET_SRC 
	# 	src/state.cpp
	src/state_context.cpp
	)

add_library(${TARGET}
	${TARGET_SRC}
	)


target_link_libraries(${TARGET}
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	)

add_dependencies(${TARGET}
	${catkin_EXPORTED_TARGETS}
	)

install(DIRECTORY include
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.hpp"
	)
