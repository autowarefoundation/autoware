cmake_minimum_required(VERSION 2.8.12)
project(vision_darknet_detect)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(autoware_config_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        autoware_config_msgs
        )

find_package(CUDA)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        autoware_config_msgs
        )

set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

AW_CHECK_CUDA()

if (USE_CUDA)
    list(APPEND CUDA_NVCC_FLAGS "--std=c++11 -I$${PROJECT_SOURCE_DIR}/darknet/src -I${PROJECT_SOURCE_DIR}/src -DGPU")
    SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

    #darknet
    cuda_add_library(vision_darknet_detect_lib SHARED
            darknet/src/activation_kernels.cu
            darknet/src/avgpool_layer_kernels.cu
            darknet/src/convolutional_kernels.cu
            darknet/src/crop_layer_kernels.cu
            darknet/src/col2im_kernels.cu
            darknet/src/blas_kernels.cu
            darknet/src/deconvolutional_kernels.cu
            darknet/src/dropout_layer_kernels.cu
            darknet/src/im2col_kernels.cu
            darknet/src/maxpool_layer_kernels.cu

            darknet/src/gemm.c
            darknet/src/utils.c
            darknet/src/cuda.c
            darknet/src/deconvolutional_layer.c
            darknet/src/convolutional_layer.c
            darknet/src/list.c
            darknet/src/image.c
            darknet/src/activations.c
            darknet/src/im2col.c
            darknet/src/col2im.c
            darknet/src/blas.c
            darknet/src/crop_layer.c
            darknet/src/dropout_layer.c
            darknet/src/maxpool_layer.c
            darknet/src/softmax_layer.c
            darknet/src/data.c
            darknet/src/matrix.c
            darknet/src/network.c
            darknet/src/connected_layer.c
            darknet/src/cost_layer.c
            darknet/src/parser.c
            darknet/src/option_list.c
            darknet/src/detection_layer.c
            darknet/src/route_layer.c
            darknet/src/upsample_layer.c
            darknet/src/box.c
            darknet/src/normalization_layer.c
            darknet/src/avgpool_layer.c
            darknet/src/layer.c
            darknet/src/local_layer.c
            darknet/src/shortcut_layer.c
            darknet/src/logistic_layer.c
            darknet/src/activation_layer.c
            darknet/src/rnn_layer.c
            darknet/src/gru_layer.c
            darknet/src/crnn_layer.c
            darknet/src/batchnorm_layer.c
            darknet/src/region_layer.c
            darknet/src/reorg_layer.c
            darknet/src/tree.c
            darknet/src/lstm_layer.c
            darknet/src/l2norm_layer.c
            darknet/src/yolo_layer.c
            )

    target_compile_definitions(vision_darknet_detect_lib PUBLIC -DGPU)
    cuda_add_cublas_to_target(vision_darknet_detect_lib)

    if (OPENMP_FOUND)
        set_target_properties(vision_darknet_detect_lib PROPERTIES
                COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
                LINK_FLAGS ${OpenMP_CXX_FLAGS}
                )
    endif ()

    target_include_directories(vision_darknet_detect_lib PRIVATE
            ${OpenCV_INCLUDE_DIR}
            ${catkin_INCLUDE_DIRS}
            ${autoware_config_msgs_INCLUDE_DIRS}
            ${autoware_msgs_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
            ${CUDA_INCLUDE_DIRS}
            ${PROJECT_SOURCE_DIR}/darknet
            ${PROJECT_SOURCE_DIR}/darknet/src
            ${PROJECT_SOURCE_DIR}/src
            )

    target_link_libraries(vision_darknet_detect_lib
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${Qt5Core_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            )

    add_dependencies(vision_darknet_detect_lib
            ${catkin_EXPORTED_TARGETS}
            )

    #ros node
    cuda_add_executable(vision_darknet_detect
            src/vision_darknet_detect_node.cpp
            src/vision_darknet_detect.cpp
            src/vision_darknet_detect.h
            )

    target_compile_definitions(vision_darknet_detect PUBLIC -DGPU)

    target_include_directories(vision_darknet_detect PRIVATE
            ${CUDA_INCLUDE_DIRS}
            ${catkin_INCLUDE_DIRS}
            ${autoware_config_msgs_INCLUDE_DIRS}
            ${autoware_msgs_INCLUDE_DIRS}
            ${PROJECT_SOURCE_DIR}/darknet
            ${PROJECT_SOURCE_DIR}/darknet/src
            ${PROJECT_SOURCE_DIR}/src
            )

    target_link_libraries(vision_darknet_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            vision_darknet_detect_lib
            )
    add_dependencies(vision_darknet_detect
            ${catkin_EXPORTED_TARGETS}
            )
    install(TARGETS vision_darknet_detect_lib vision_darknet_detect
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

ELSE()
    message(WARNING "Building vision_darknet_detect without CUDA")
	
    add_library(vision_darknet_detect_lib SHARED
            darknet/src/gemm.c
            darknet/src/utils.c
            darknet/src/cuda.c
            darknet/src/deconvolutional_layer.c
            darknet/src/convolutional_layer.c
            darknet/src/list.c
            darknet/src/image.c
            darknet/src/activations.c
            darknet/src/im2col.c
            darknet/src/col2im.c
            darknet/src/blas.c
            darknet/src/crop_layer.c
            darknet/src/dropout_layer.c
            darknet/src/maxpool_layer.c
            darknet/src/softmax_layer.c
            darknet/src/data.c
            darknet/src/matrix.c
            darknet/src/network.c
            darknet/src/connected_layer.c
            darknet/src/cost_layer.c
            darknet/src/parser.c
            darknet/src/option_list.c
            darknet/src/detection_layer.c
            darknet/src/route_layer.c
            darknet/src/upsample_layer.c
            darknet/src/box.c
            darknet/src/normalization_layer.c
            darknet/src/avgpool_layer.c
            darknet/src/layer.c
            darknet/src/local_layer.c
            darknet/src/shortcut_layer.c
            darknet/src/logistic_layer.c
            darknet/src/activation_layer.c
            darknet/src/rnn_layer.c
            darknet/src/gru_layer.c
            darknet/src/crnn_layer.c
            darknet/src/batchnorm_layer.c
            darknet/src/region_layer.c
            darknet/src/reorg_layer.c
            darknet/src/tree.c
            darknet/src/lstm_layer.c
            darknet/src/l2norm_layer.c
            darknet/src/yolo_layer.c
            )

    target_include_directories(vision_darknet_detect_lib PRIVATE
            ${OpenCV_INCLUDE_DIR}
            ${catkin_INCLUDE_DIRS}
            ${autoware_config_msgs_INCLUDE_DIRS}
            ${autoware_msgs_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
            ${PROJECT_SOURCE_DIR}/darknet
            ${PROJECT_SOURCE_DIR}/darknet/src
            ${PROJECT_SOURCE_DIR}/src
            )

    target_link_libraries(vision_darknet_detect_lib
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${Qt5Core_LIBRARIES}
            m
            pthread
            )

    add_dependencies(vision_darknet_detect_lib
            ${catkin_EXPORTED_TARGETS}
            )

    #ros node
    add_executable(vision_darknet_detect
            src/vision_darknet_detect_node.cpp
            src/vision_darknet_detect.cpp
            src/vision_darknet_detect.h
            )

    target_include_directories(vision_darknet_detect PRIVATE
            ${catkin_INCLUDE_DIRS}
            ${autoware_config_msgs_INCLUDE_DIRS}
            ${autoware_msgs_INCLUDE_DIRS}
            ${PROJECT_SOURCE_DIR}/darknet
            ${PROJECT_SOURCE_DIR}/darknet/src
            ${PROJECT_SOURCE_DIR}/src
            )

    target_link_libraries(vision_darknet_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS}
            vision_darknet_detect_lib
            )
    add_dependencies(vision_darknet_detect
            ${catkin_EXPORTED_TARGETS}
            )
    install(TARGETS vision_darknet_detect_lib vision_darknet_detect
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
    
ENDIF ()

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY darknet/cfg/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/darknet/cfg/
        PATTERN ".svn" EXCLUDE)
