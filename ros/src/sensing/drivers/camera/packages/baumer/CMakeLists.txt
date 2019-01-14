cmake_minimum_required(VERSION 2.8.3)
project(vlg22c_cam)

find_package(catkin REQUIRED COMPONENTS 
    autoware_build_flags
    image_transport
    roscpp
    std_msgs
    sensor_msgs
    camera_info_manager)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
    LIBRARIES ${PROJECT_NAME}
)

set(BAUMER_INC_DIR "$ENV{HOME}/Baumer/BaumerGAPI/Components/Dev/C_C++/Inc")
set(BAUMER_LIB_DIR "$ENV{HOME}/Baumer/BaumerGAPI/Components/Bin/Systems/GigE")

if(EXISTS "${BAUMER_INC_DIR}")

add_definitions(-D_GNULINUX)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${BAUMER_INC_DIR}
    nodes/vlg22/src/
    ${OpenCV_INCLUDE_DIRS}
)

link_directories("${BAUMER_LIB_DIR}")

add_executable(${PROJECT_NAME}_node
    nodes/vlg22/src/vlg22_main.cpp
    nodes/vlg22/src/bgapi_init.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    bgapi
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)

add_custom_command(TARGET ${PROJECT_NAME}_node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${BAUMER_LIB_DIR} $<TARGET_FILE_DIR:${PROJECT_NAME}_node>
)

else()
    message("'BGGAPI' is not installed. 'vlg22c_cam' will not be built.")
endif()
