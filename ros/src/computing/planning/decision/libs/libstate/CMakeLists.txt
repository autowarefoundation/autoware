cmake_minimum_required(VERSION 2.8.3)
project(state)

# set cpp-11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES state
        CATKIN_DEPENDS roscpp
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)

add_library(${PROJECT_NAME}
        src/state_context.cpp
	)

#target_link_libraries(${TARGET}
#	${catkin_LIBRARIES}
#	${OpenCV_LIBS}
#	)

add_dependencies(${PROJECT_NAME}
	${catkin_EXPORTED_TARGETS}
	)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
