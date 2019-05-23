cmake_minimum_required(VERSION 2.8.12)
project(vision_segment_enet_detect)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        autoware_build_flags
        )
FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        )
###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

INCLUDE_DIRECTORIES(
        ${catkin_INCLUDE_DIRS}
)


#####ENET########
##############################ENet's CAFFE FORK NEEDS TO BE PREVIOUSLY COMPILED####################
set(ENET_CAFFE_PATH "$ENV{HOME}/ENet/caffe-enet/distribute")
####################################################################################################
AW_CHECK_CUDA()

if (USE_CUDA AND EXISTS "${ENET_CAFFE_PATH}")

    ADD_EXECUTABLE(vision_segment_enet_detect
            nodes/vision_segment_enet_detect/vision_segment_enet_detect_node.cpp
            nodes/vision_segment_enet_detect/vision_segment_enet_detect.cpp
            )

    TARGET_LINK_LIBRARIES(vision_segment_enet_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${ENET_CAFFE_PATH}/lib/libcaffe.so
            glog
            )

    TARGET_INCLUDE_DIRECTORIES(vision_segment_enet_detect PRIVATE
            ${CUDA_INCLUDE_DIRS}
            ${ENET_CAFFE_PATH}/include
            include
            )

    ADD_DEPENDENCIES(vision_segment_enet_detect
            ${catkin_EXPORTED_TARGETS}
            )

    install(TARGETS
            vision_segment_enet_detect
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            PATTERN ".svn" EXCLUDE)

else ()
    message("' ENet/Caffe' is not installed. 'vision_segment_enet_detect' will not be built.")
endif ()
