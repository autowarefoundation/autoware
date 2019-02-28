cmake_minimum_required(VERSION 2.8.3)
project(kvaser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        visualization_msgs
        autoware_can_msgs
        )

catkin_package(
        CATKIN_DEPENDS std_msgs visualization_msgs autoware_can_msgs
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)

if (EXISTS "/usr/include/canlib.h")

    add_executable(can_listener
            nodes/can_listener/can_listener.cpp
            )
    target_include_directories(can_listener PRIVATE ${catkin_INCLUDE_DIRS})

    target_link_libraries(can_listener ${catkin_LIBRARIES})

    target_link_libraries(can_listener canlib)

    add_dependencies(can_listener ${catkin_EXPORTED_TARGETS})
    install(TARGETS can_listener
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else ()
    message("'canlib' is not installed. 'can_listener' is not built.")
endif ()

add_executable(can_converter
        nodes/can_converter/can_converter.cpp)
target_include_directories(can_converter PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(can_converter ${catkin_LIBRARIES})
add_dependencies(can_converter ${catkin_EXPORTED_TARGETS})

add_executable(can_draw
        nodes/can_draw/can_draw.cpp
        )
target_include_directories(can_draw PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(can_draw ${catkin_LIBRARIES})
add_dependencies(can_draw ${catkin_EXPORTED_TARGETS})

install(TARGETS can_converter can_draw
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
