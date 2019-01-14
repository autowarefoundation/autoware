cmake_minimum_required(VERSION 2.8.3)
project(kf_lib)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        )

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs autoware_msgs
        INCLUDE_DIRS include
        LIBRARIES kf_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(${catkin_INCLUDE_DIRS}
        include)

## Declare a cpp library
add_library(kf_lib
        src/kf_lib.cpp
        )

target_link_libraries(kf_lib
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_dependencies(kf_lib ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


install(TARGETS kf_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
