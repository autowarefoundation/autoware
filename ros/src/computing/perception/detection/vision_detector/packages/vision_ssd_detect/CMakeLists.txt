cmake_minimum_required(VERSION 2.8.12)
project(vision_ssd_detect)

set(ROS_VERSION $ENV{ROS_DISTRO})

include(FindPkgConfig)


FIND_PACKAGE(catkin REQUIRED COMPONENTS
        autoware_build_flags
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        tf
        jsk_recognition_msgs
        )

FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        tf
        jsk_recognition_msgs
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
)

IF (CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})

    IF ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
        link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
    endif ()
ENDIF ()


#####SSD########
##############################SSD'sFORK of CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
set(SSD_CAFFE_PATH "$ENV{HOME}/ssdcaffe/distribute")
####################################################################################################
if (EXISTS "${SSD_CAFFE_PATH}")

    add_executable(vision_ssd_detect
            src/vision_ssd_detect.cpp
            src/vision_ssd_detect_node.cpp
            )

    target_link_libraries(vision_ssd_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${SSD_CAFFE_PATH}/lib/libcaffe.so
            glog
            )

    target_include_directories(vision_ssd_detect PRIVATE
            ${SSD_CAFFE_PATH}/include
            include
            )

    add_dependencies(vision_ssd_detect
            ${catkin_EXPORTED_TARGETS}
            )

    install(TARGETS vision_ssd_detect
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
else ()
    message("'SSD/Caffe' is not installed. 'ssd_unc' will not be built.")
endif ()
