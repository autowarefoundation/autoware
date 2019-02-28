cmake_minimum_required(VERSION 2.8.3)
project(op_simu)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Get a ros version
set(ROS_VERSION $ENV{ROS_DISTRO})


find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        op_utility
        op_planner
        )

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(X11 REQUIRED)
find_package(Threads REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES op_simu
        CATKIN_DEPENDS op_utility op_planner
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS}
        include
        ${OpenCV_INCLUDE_DIRS}
        ${OpenGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
        )

set(SIMUH_SRC        
        src/SimpleTracker.cpp
        src/SimulatedTrajectoryFollower.cpp
        src/TrajectoryFollower.cpp        
        )

## Declare a cpp library
add_library(${PROJECT_NAME}
        ${SIMUH_SRC}
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${X11_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        )
add_executable(Simu
        ${SIMUH_SRC}
        main.cpp
        )

target_link_libraries(Simu
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${GLUT_LIBRARY}
        ${GLEW_LIBRARIES}
        ${X11_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        )


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


install(TARGETS op_simu Simu
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
