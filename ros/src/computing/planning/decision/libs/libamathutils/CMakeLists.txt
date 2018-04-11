cmake_minimum_required(VERSION 2.8.11)
project (amathutils) # autoware math utility

# set cpp-11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} amathutils
    CATKIN_DEPENDS roscpp
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}

)

add_library(${PROJECT_NAME}
        src/Amathutils.cpp
	)

add_dependencies(${PROJECT_NAME}
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(amathutils ${catkin_LIBRARIES})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn"
        EXCLUDE
)

install(TARGETS amathutils
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


