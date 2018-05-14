cmake_minimum_required(VERSION 2.8.12)
project(cv_tracker)

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
        lib
        include
)

IF (CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})

    IF ("${ARCHITECTURE}" MATCHES "^arm")
        link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
    endif ()
ENDIF ()

## dpm_ttic ##


#this must be fixed to use cmake cuda commands
IF (CUDA_FOUND)

    cuda_add_executable(dpm_ttic
            nodes/dpm_ttic/dpm_ttic.cpp
            )

    target_link_libraries(dpm_ttic
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            )

    set_target_properties(dpm_ttic
            PROPERTIES
            COMPILE_FLAGS
            "-DHAS_GPU -DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/GPU_function.cubin -DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
            )
ELSE ()

    add_executable(dpm_ttic
            nodes/dpm_ttic/dpm_ttic.cpp
            )

    target_link_libraries(dpm_ttic
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            )

    set_target_properties(dpm_ttic
            PROPERTIES
            COMPILE_FLAGS
            "-DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
            )
ENDIF ()


add_dependencies(dpm_ttic
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS dpm_ttic
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## dpm_ocv
if ("${ROS_VERSION}" MATCHES "(indigo|jade)")

    IF (CUDA_FOUND)
        cuda_add_executable(dpm_ocv
                nodes/dpm_ocv/dpm_ocv.cpp
                )
        set_target_properties(dpm_ocv
                PROPERTIES
                COMPILE_FLAGS
                "-DHAS_GPU -DDEFAULT_CUBIN=${CATKIN_DEVEL_PREFIX}/lib/gpu_function.cubin -DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/model/models_VOC2007/"
                )
    ELSE ()
        add_executable(dpm_ocv
                nodes/dpm_ocv/dpm_ocv.cpp
                )
        set_target_properties(dpm_ocv
                PROPERTIES
                COMPILE_FLAGS
                "-DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
                )
    ENDIF ()

    target_link_libraries(dpm_ocv
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            )

    add_dependencies(dpm_ocv
            ${catkin_EXPORTED_TARGETS}
            )

    install(TARGETS dpm_ocv
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

ENDIF ()
#ROS_VERSION_ENDIF


## kf_tracker ##
#if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
#    ADD_EXECUTABLE(kf_track
#            nodes/kf_track/kf_track.cpp
#            )

#    TARGET_LINK_LIBRARIES(kf_track
#            ${catkin_LIBRARIES}
#            ${OpenCV_LIBRARIES}
#            )
#    add_dependencies(kf_track ${catkin_EXPORTED_TARGETS})
#endif ()

## dummy_tracker ##
add_executable(dummy_track
        nodes/dummy_track/dummy_track.cpp
        )
target_link_libraries(dummy_track
        ${catkin_LIBRARIES}
        )
install(TARGETS dummy_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )


## klt_tracker ##
add_executable(klt_track
        nodes/klt_track/klt_track.cpp
        )

include_directories(lib/lktracker)

add_library(lktracker
        lib/lktracker/LkTracker.cpp
        )
target_link_libraries(lktracker
        ${OpenCV_LIBRARIES}
        )
add_dependencies(lktracker
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(klt_track
        lktracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_dependencies(klt_track
        ${catkin_EXPORTED_TARGETS}
        )
install(TARGETS klt_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

############# RCNN#############

###########################################CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
##############DONT FORGET TO INSTALL fastrcnn's caffe dynamic libraries to /usr/local/lib	 	###
set(RCNN_CAFFE_PATH "$ENV{HOME}/fast-rcnn/caffe-fast-rcnn/distribute")
####################################################################################################

if (EXISTS "${RCNN_CAFFE_PATH}")
    find_package(librcnn)

    add_executable(rcnn_msr
            nodes/rcnn/rcnn_node.cpp
            )

    target_link_libraries(rcnn_msr
            librcnn
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            )

    target_include_directories(rcnn_msr PRIVATE
            ${librcnn_INCLUDE_DIRS}
            ${RCNN_CAFFE_PATH}/include
            ${catkin_INCLUDE_DIRS}
            )

    add_dependencies(rcnn_msr
            ${catkin_EXPORTED_TARGETS}
            librcnn
            )
    install(TARGETS rcnn_msr
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
else ()
    message("'FastRCNN/Caffe' are not installed. 'rcnn_msr' will not be built.")
endif ()

#####SSD########
##############################SSD'sFORK of CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
set(SSD_CAFFE_PATH "$ENV{HOME}/ssdcaffe/distribute")
####################################################################################################
if (EXISTS "${SSD_CAFFE_PATH}")

    add_executable(ssd_unc
            nodes/ssd/ssd_node.cpp
            nodes/ssd/ssd_detector.cpp
            )

    target_link_libraries(ssd_unc
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${SSD_CAFFE_PATH}/lib/libcaffe.so
            glog
            )

    target_include_directories(ssd_unc PRIVATE
            ${SSD_CAFFE_PATH}/include
            nodes/ssd/include
            )

    add_dependencies(ssd_unc
            ${catkin_EXPORTED_TARGETS}
            )

    install(TARGETS ssd_unc
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
else ()
    message("'SSD/Caffe' is not installed. 'ssd_unc' will not be built.")
endif ()

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

    cuda_add_executable(yolo2_wa
            nodes/yolo2/src/yolo2_node.cpp
            nodes/yolo2/src/darknet/yolo2.cpp
            )

    target_compile_definitions(yolo2_wa PUBLIC -DGPU)

    target_include_directories(yolo2_wa PRIVATE
            nodes/yolo2/include
            ${YOLO2_PATH}/include
            ${YOLO2_PATH}/src
            ${CUDA_INCLUDE_DIRS}
            )

    target_link_libraries(yolo2_wa
            yolo2lib_wa
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            )

    install(TARGETS yolo2_wa
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )


else ()
    message("'Darknet' was not found. 'yolo2_wa' will not be built.")
endif ()
