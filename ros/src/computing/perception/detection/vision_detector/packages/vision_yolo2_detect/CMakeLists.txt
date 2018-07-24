cmake_minimum_required(VERSION 2.8.12)
project(vision_yolo2_detect)

set(ROS_VERSION $ENV{ROS_DISTRO})

include(FindPkgConfig)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
        autoware_build_flags
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        autoware_msgs
        )

FIND_PACKAGE(CUDA)
FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS cv_bridge
        image_transport
        roscpp
        sensor_msgs
        autoware_msgs
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        lib
        include
)

IF (CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})

    IF ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
        link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
    endif ()
ENDIF ()


###YOLO2####
###########YOLO2 PATH#####################################
set(YOLO2_PATH "$ENV{HOME}/darknet")
##########################################################
if (EXISTS "${YOLO2_PATH}")

    set(YOLO2_USE_CUDNN 0)
    set(YOLO2_USE_GPU 1)

    list(APPEND CUDA_NVCC_FLAGS "--std=c++11 -I${YOLO2_PATH}/include -I${YOLO2_PATH}/src")
    set(CUDA_PROPAGATE_HOST_FLAGS OFF)


    cuda_add_library(yolo2lib_wa
            ${YOLO2_PATH}/src/activation_kernels.cu
            ${YOLO2_PATH}/src/activation_layer.c
            ${YOLO2_PATH}/src/activations.c
            ${YOLO2_PATH}/src/avgpool_layer.c
            ${YOLO2_PATH}/src/avgpool_layer_kernels.cu
            ${YOLO2_PATH}/src/batchnorm_layer.c
            ${YOLO2_PATH}/src/blas.c
            ${YOLO2_PATH}/src/blas_kernels.cu
            ${YOLO2_PATH}/src/box.c
            ${YOLO2_PATH}/src/col2im.c
            ${YOLO2_PATH}/src/col2im_kernels.cu
            ${YOLO2_PATH}/src/connected_layer.c
            ${YOLO2_PATH}/src/convolutional_layer.c
            ${YOLO2_PATH}/src/convolutional_kernels.cu
            ${YOLO2_PATH}/src/deconvolutional_layer.c
            ${YOLO2_PATH}/src/deconvolutional_kernels.cu
            ${YOLO2_PATH}/src/cost_layer.c
            ${YOLO2_PATH}/src/crnn_layer.c
            ${YOLO2_PATH}/src/crop_layer.c
            ${YOLO2_PATH}/src/crop_layer_kernels.cu
            ${YOLO2_PATH}/src/cuda.c
            ${YOLO2_PATH}/src/data.c
            ${YOLO2_PATH}/src/detection_layer.c
            ${YOLO2_PATH}/src/dropout_layer.c
            ${YOLO2_PATH}/src/dropout_layer_kernels.cu
            ${YOLO2_PATH}/src/gemm.c
            ${YOLO2_PATH}/src/gru_layer.c
            ${YOLO2_PATH}/src/im2col.c
            ${YOLO2_PATH}/src/im2col_kernels.cu
            ${YOLO2_PATH}/src/image.c
            ${YOLO2_PATH}/src/layer.c
            ${YOLO2_PATH}/src/list.c
            ${YOLO2_PATH}/src/local_layer.c
            ${YOLO2_PATH}/src/matrix.c
            ${YOLO2_PATH}/src/maxpool_layer.c
            ${YOLO2_PATH}/src/maxpool_layer_kernels.cu
            ${YOLO2_PATH}/src/network.c
            ${YOLO2_PATH}/src/network_kernels.cu
            ${YOLO2_PATH}/src/normalization_layer.c
            ${YOLO2_PATH}/src/option_list.c
            ${YOLO2_PATH}/src/parser.c
            ${YOLO2_PATH}/src/region_layer.c
            ${YOLO2_PATH}/src/reorg_layer.c
            ${YOLO2_PATH}/src/rnn_layer.c
            ${YOLO2_PATH}/src/route_layer.c
            ${YOLO2_PATH}/src/shortcut_layer.c
            ${YOLO2_PATH}/src/softmax_layer.c
            ${YOLO2_PATH}/src/tree.c
            ${YOLO2_PATH}/src/utils.c

            OPTIONS
            -DGPU=${YOLO2_USE_GPU}
            #    -DCUDNN=${YOLO2_USE_CUDNN}
            )

    cuda_add_cublas_to_target(yolo2lib_wa)

    target_compile_definitions(yolo2lib_wa PUBLIC -DGPU)

    target_include_directories(yolo2lib_wa PRIVATE
            ${YOLO2_PATH}/include
            ${YOLO2_PATH}/src
            ${CUDA_INCLUDE_DIRS}
            )

    target_link_libraries(yolo2lib_wa
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            )

    cuda_add_executable(vision_yolo2_detect
            src/vision_yolo2_detect.cpp
            src/darknet/yolo2.cpp
            )

    target_compile_definitions(vision_yolo2_detect PUBLIC -DGPU)

    target_include_directories(vision_yolo2_detect PRIVATE
            nodes/yolo2/include
            ${YOLO2_PATH}/include
            ${YOLO2_PATH}/src
            ${CUDA_INCLUDE_DIRS}
            )

    target_link_libraries(vision_yolo2_detect
            yolo2lib_wa
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            )

    install(TARGETS vision_yolo2_detect
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )


else ()
    message("'Darknet' was not found. 'vision_yolo2_detect' will not be built.")
endif ()
