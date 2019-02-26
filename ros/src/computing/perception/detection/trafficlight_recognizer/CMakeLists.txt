cmake_minimum_required(VERSION 3.5)
project(trafficlight_recognizer)
execute_process(
        COMMAND rosversion -d
        OUTPUT_VARIABLE ROS_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(ROS_VERSION $ENV{ROS_DISTRO})


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(vector_map REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(libvectormap REQUIRED)
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf REQUIRED)
find_package(vector_map_server REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(waypoint_follower REQUIRED)
find_package(ament_cmake REQUIRED)

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
        ${cv_bridge_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${libvectormap_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
        ${waypoint_follower_INCLUDE_DIRS}
        ${vector_map_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-O2 -Wall -Wunused-variable ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########
add_library(libcontext lib/Context.cpp)
add_dependencies(libcontext
        ${catkin_EXPORTED_TARGETS}
        )
ament_target_dependencies(libcontext
        autoware_msgs
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

### tl_switch ###
add_executable(tl_switch
        nodes/tl_switch/tl_switch.cpp
        )
ament_target_dependencies(tl_switch
        autoware_msgs
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

### region_tlr ###
add_executable(region_tlr
        nodes/region_tlr/region_tlr.cpp
        nodes/region_tlr/TrafficLightDetector.cpp
        )

target_link_libraries(region_tlr
        ${OpenCV_LIBS}
        libcontext
        ${OPENGL_LIBRARIES}
        )

ament_target_dependencies(region_tlr
        autoware_msgs
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

### feat_proj ###
include_directories(
        ${cv_bridge_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${libvectormap_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
        ${waypoint_follower_INCLUDE_DIRS}
        ${vector_map_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

add_executable(feat_proj
        nodes/feat_proj/feat_proj.cpp
        )

target_link_libraries(feat_proj
        ${OpenCV_LIBS}
        )

ament_target_dependencies(feat_proj
        autoware_msgs
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

### tlr_tuner ###
EXECUTE_PROCESS(
        COMMAND pkg-config --variable=host_bins Qt5Core
        OUTPUT_VARIABLE Qt5BIN
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
        include
        nodes/tlr_tuner/
        ${cv_bridge_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${libvectormap_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
        ${waypoint_follower_INCLUDE_DIRS}
        ${vector_map_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
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
        ${OpenCV_LIBS}
        Qt5::Core
        Qt5::Widgets
        ${OPENGL_LIBRARIES}
        )

ament_target_dependencies(tlr_tuner
        autoware_msgs
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


### roi_extractor ###
include_directories(
        ${cv_bridge_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${libvectormap_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
        ${waypoint_follower_INCLUDE_DIRS}
        ${vector_map_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        include
)

add_executable(roi_extractor
        nodes/roi_extractor/roi_extractor.cpp
        )

target_link_libraries(roi_extractor
        libcontext
        )

ament_target_dependencies(roi_extractor
        autoware_msgs
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


### label_maker ###
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(TinyXML REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
        ${cv_bridge_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${libvectormap_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
        ${waypoint_follower_INCLUDE_DIRS}
        ${vector_map_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
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
        ${TinyXML_LIBRARIES}
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        ${OPENGL_LIBRARIES}
        )

ament_target_dependencies(label_maker
        autoware_msgs
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

    ament_target_dependencies(region_tlr_ssd
            autoware_msgs
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

    install(TARGETS region_tlr_ssd
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION lib/${PROJECT_NAME}
            )

else ()                          # if(EXISTS "${SSD_CAFFE_PATH}")
    message("'SSD/Caffe' is not installed. 'region_tlr_ssd' will not be built.")
endif ()                         # if(EXISTS "${SSD_CAFFE_PATH}")


install(TARGETS region_tlr feat_proj tlr_tuner roi_extractor label_maker
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
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

    ament_target_dependencies(region_tlr_mxnet
            autoware_msgs
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

    install(TARGETS region_tlr_mxnet
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION lib/${PROJECT_NAME}
            )

else ()
    message("'MXNET Package' is not installed. 'region_tlr_mxnet' will not be built.")
endif ()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(autoware_msgs)
ament_export_dependencies(cmake_modules)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(libvectormap)
ament_export_dependencies(roscpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(tf)
ament_export_dependencies(vector_map)
ament_export_dependencies(vector_map_server)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(waypoint_follower)

ament_export_include_directories(include)
ament_export_include_directories(${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(libcontext)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()