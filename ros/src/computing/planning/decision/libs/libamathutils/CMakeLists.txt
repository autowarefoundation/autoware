cmake_minimum_required(VERSION 2.8.11)
project (amathutils) # autoware math utility
find_package(catkin REQUIRED COMPONENTS
	roscpp
	autoware_msgs
)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

set(TARGET ${PROJECT_NAME})
set(TARGET_SRC  src/Amathutils.cpp
	)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES amathutils
	CATKIN_DEPENDS roscpp autoware_msgs
)

include_directories(${TARGET} 
	include
	${catkin_INCLUDE_DIRS}
	)

add_library(${TARGET}
	${TARGET_SRC}
	)
target_link_libraries(${TARGET}
	${CMAKE_THREAD_LIBS_INIT}
	)

add_dependencies(${TARGET}
	${catkin_EXPORTED_TARGETS}
	)

install(DIRECTORY include
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.hpp"
	)
