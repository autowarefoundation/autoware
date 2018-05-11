cmake_minimum_required(VERSION 2.8.12)
project(range_fusion)

set(ROS_VERSION $ENV{ROS_DISTRO})

include(FindPkgConfig)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
    FIND_PACKAGE(catkin REQUIRED COMPONENTS
            cv_bridge
            image_transport
            roscpp
            sensor_msgs
            std_msgs
            autoware_msgs
            libdpm_ttic
            fusion
            tf
            libdpm_ocv
            #kf_lib
            jsk_recognition_msgs
            gnss
            )
elseif ("${ROS_VERSION}" MATCHES "(kinetic)")
    FIND_PACKAGE(catkin REQUIRED COMPONENTS
            cv_bridge
            image_transport
            roscpp
            std_msgs
            autoware_msgs
            sensor_msgs
            libdpm_ttic
            fusion
            tf
            #kf_lib
            jsk_recognition_msgs
            gnss
            )
endif ()

FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

EXECUTE_PROCESS(
        COMMAND uname -m
        OUTPUT_VARIABLE ARCHITECTURE
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
    catkin_package(
            CATKIN_DEPENDS cv_bridge
            image_transport
            roscpp
            sensor_msgs
            std_msgs
            autoware_msgs
            libdpm_ttic
            fusion
            tf
            libdpm_ocv
            #kf_lib
            jsk_recognition_msgs
            gnss
    )
elseif ("${ROS_VERSION}" MATCHES "(kinetic)")
    catkin_package(
            CATKIN_DEPENDS cv_bridge
            image_transport
            roscpp
            std_msgs
            autoware_msgs
            sensor_msgs
            libdpm_ttic
            fusion
            tf
            #kf_lib
            jsk_recognition_msgs
            gnss
    )
endif ()
###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
)



## range_fusion ##
add_executable(range_fusion
        range_fusion.cpp
        )

target_link_libraries(range_fusion
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_dependencies(range_fusion
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS range_fusion
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
