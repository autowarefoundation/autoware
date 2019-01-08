cmake_minimum_required(VERSION 2.8.3)
project(trafficlight_recognizer)
execute_process(
        COMMAND rosversion -d
        OUTPUT_VARIABLE ROS_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(ROS_VERSION $ENV{ROS_DISTRO})

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        autoware_msgs
        cmake_modules
        cv_bridge
        geometry_msgs
        libvectormap
        roscpp
        sensor_msgs
        std_msgs
        tf
        vector_map
        vector_map_server
        visualization_msgs
        waypoint_follower
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(OpenGL REQUIRED)

###################################
## catkin specific configuration ##
###################################
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
        CATKIN_DEPENDS
        autoware_msgs
        cmake_modules
        cv_bridge
        geometry_msgs
        libvectormap
        roscpp
        sensor_msgs
        std_msgs
        tf
        vector_map
        vector_map_server
        visualization_msgs
        waypoint_follower

        LIBRARIES libcontext
)

set(CMAKE_CXX_FLAGS "-O2 -Wall -Wunused-variable ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########
add_library(libcontext lib/Context.cpp)
add_dependencies(libcontext
        ${catkin_EXPORTED_TARGETS}
        )

### tl_switch ###
add_executable(tl_switch
        nodes/tl_switch/tl_switch.cpp
        )

target_link_libraries(tl_switch
        ${catkin_LIBRARIES}
        )

add_dependencies(tl_switch
        ${catkin_EXPORTED_TARGETS}
        )

### region_tlr ###
add_executable(region_tlr
        nodes/region_tlr/region_tlr.cpp
        nodes/region_tlr/TrafficLightDetector.cpp
        )

target_link_libraries(region_tlr
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        libcontext
        ${OPENGL_LIBRARIES}
        )

add_dependencies(region_tlr
        ${catkin_EXPORTED_TARGETS}
        )

### feat_proj ###
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

add_executable(feat_proj
        nodes/feat_proj/feat_proj.cpp
        )

target_link_libraries(feat_proj
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_dependencies(feat_proj
        ${catkin_EXPORTED_TARGETS}
        )

### tlr_tuner ###
EXECUTE_PROCESS(
        COMMAND pkg-config --variable=host_bins Qt5Core
        OUTPUT_VARIABLE Qt5BIN
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
        include
        nodes/tlr_tuner/
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

qt5_wrap_ui(tlr_tuner_ui_mainwindow nodes/tlr_tuner/mainwindow.ui)

add_executable(tlr_tuner
        nodes/tlr_tuner/tlr_tuner.cpp
        nodes/tlr_tuner/mainwindow.cpp
        nodes/tlr_tuner/mainwindow.h
        nodes/tlr_tuner/tunerBody.cpp
        nodes/tlr_tuner/tunerBody.h
        ${tlr_tuner_ui_mainwindow}
        )

set_target_properties(tlr_tuner
        PROPERTIES COMPILE_FLAGS "-fPIC"
        )

target_link_libraries(tlr_tuner
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        Qt5::Core
        Qt5::Widgets
        ${OPENGL_LIBRARIES}
        )


### roi_extractor ###
include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

add_executable(roi_extractor
        nodes/roi_extractor/roi_extractor.cpp
        )

target_link_libraries(roi_extractor
        ${catkin_LIBRARIES}
        libcontext
        )

add_dependencies(region_tlr
        ${catkin_EXPORTED_TARGETS})


### label_maker ###
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(TinyXML REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}
        nodes/label_maker/
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

file(GLOB SOURCES "nodes/label_maker/*.cpp")
file(GLOB HEADERS "nodes/label_maker/*.h")
file(GLOB UI_SOURCES "nodes/label_maker/*.ui")

qt5_wrap_ui(UI_HEADERS ${UI_SOURCES})

add_executable(label_maker
        ${SOURCES}
        ${HEADERS}
        ${UI_HEADERS}
        )

set_target_properties(label_maker
        PROPERTIES COMPILE_FLAGS "-fPIC"
        )

target_link_libraries(label_maker
        ${catkin_LIBRARIES}
        ${TinyXML_LIBRARIES}
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        ${OPENGL_LIBRARIES}
        )


### region_tlr_ssd ###
##############################SSD'sFORK of CAFFE NEEDS TO BE PREVIOUSLY COMPILED####################
set(SSD_CAFFE_PATH "$ENV{HOME}/ssdcaffe/distribute")
####################################################################################################
if (EXISTS "${SSD_CAFFE_PATH}")

    find_package(CUDA REQUIRED)

    add_executable(region_tlr_ssd
            nodes/region_tlr_ssd/region_tlr_ssd.cpp
            nodes/region_tlr_ssd/traffic_light_recognizer.cpp
            )

    target_link_libraries(region_tlr_ssd
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${SSD_CAFFE_PATH}/lib/libcaffe.so
            glog
            libcontext
            )

    target_include_directories(region_tlr_ssd PRIVATE
            ${SSD_CAFFE_PATH}/include
            ${CUDA_INCLUDE_DIRS}
            nodes/region_tlr_ssd/
            )

    add_dependencies(region_tlr_ssd
            ${catkin_EXPORTED_TARGETS}
            )

    install(TARGETS region_tlr_ssd
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

else ()                          # if(EXISTS "${SSD_CAFFE_PATH}")
    message("'SSD/Caffe' is not installed. 'region_tlr_ssd' will not be built.")
endif ()                         # if(EXISTS "${SSD_CAFFE_PATH}")


install(TARGETS region_tlr feat_proj tlr_tuner roi_extractor label_maker
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)


### region_tlr_mxnet ###
##############################MXNET with CPP_PACKAGE NEEDS TO BE PREVIOUSLY COMPILED################
#############Please follow README file for instructions#############################################
set(MXNET_PATH "$ENV{HOME}/mxnet/")
####################################################################################################
if (EXISTS "${MXNET_PATH}")

    find_package(CUDA REQUIRED)

    add_executable(region_tlr_mxnet
            nodes/region_tlr_mxnet/region_tlr_mxnet.cpp
            nodes/region_tlr_mxnet/mxnet_traffic_light_recognizer.cpp
            nodes/region_tlr_mxnet/region_tlr_mxnet.h
            nodes/region_tlr_mxnet/mxnet_traffic_light_recognizer.h
            )

    target_link_libraries(region_tlr_mxnet
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${MXNET_PATH}/lib/libmxnet.so
            libcontext
            )

    target_include_directories(region_tlr_mxnet PRIVATE
            ${MXNET_PATH}/nnvm/include/
            ${MXNET_PATH}/cpp-package/include/
            ${MXNET_PATH}/include/
            ${MXNET_PATH}/dmlc-core/include/
            ${CUDA_INCLUDE_DIRS}
            nodes/region_tlr_mxnet/
            include
            )

    add_dependencies(region_tlr_mxnet
            ${catkin_EXPORTED_TARGETS}
            )

    install(TARGETS region_tlr_mxnet
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

else ()
    message("'MXNET Package' is not installed. 'region_tlr_mxnet' will not be built.")
endif ()
