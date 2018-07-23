cmake_minimum_required(VERSION 2.8.3)
project(orb_localizer)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        tf
        sensor_msgs
        image_transport
        cv_bridge
        message_generation
        std_msgs
        )

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# Force using C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(Boost REQUIRED COMPONENTS system serialization python)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenMP)

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

find_package(PCL 1.7 REQUIRED COMPONENTS common octree)
find_package(PythonLibs REQUIRED)
find_package(X11 REQUIRED)

add_message_files(
        FILES
        debug.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
        CATKIN_DEPENDS message_runtime
        DEPENDS OpenCV GLEW PCL
)

# Vocabulary file
set(orb_slam_vocabulary_file ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/ORBvoc.txt)
add_custom_target(orb_vocabulary ALL
        [ ! -e ${orb_slam_vocabulary_file} ] && tar -zxf ${PROJECT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz --directory ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME} || return 0
        DEPENDS Vocabulary/ORBvoc.txt.tar.gz
        )
add_definitions(
        -DORB_SLAM_VOCABULARY="${orb_slam_vocabulary_file}"
)

# For building Pangolin
add_subdirectory(Thirdparty/Pangolin)

# For building DBoW2
add_subdirectory(Thirdparty/DBoW2)

# For building g2o
add_subdirectory(Thirdparty/g2o)


include_directories(
        # my own
        include
        ${catkin_INCLUDE_DIRS}

        # Important libraries
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIR}
        ${Eigen3_INCLUDE_DIRS}

        # Third parties
        Thirdparty
        Thirdparty/g2o
        Thirdparty/Pangolin/include

)


add_library(
        orb_slam2 STATIC
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
)


list(APPEND LINK_LIBRARIES
        boost_system
        boost_serialization
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${X11_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${PCL_LIBRARIES}
        ${GLUT_LIBRARIES}
        )

list(APPEND ORB_BIN_LINKS
        orb_slam2
        DBoW2
        g2o
        pangolin
        )


# Main Executables
add_executable(
        orb_mapping
        nodes/orb_mapping/orb_mapping.cpp
        nodes/common.cpp
)

target_link_libraries(
        orb_mapping
        ${ORB_BIN_LINKS}
        ${LINK_LIBRARIES}
)

add_executable(
        orb_matching
        nodes/orb_matching/orb_matching.cpp
        nodes/orb_matching/Matcher.cpp
        nodes/common.cpp
)

add_dependencies(orb_matching ${catkin_EXPORTED_TARGETS})

target_link_libraries(
        orb_matching
        ${ORB_BIN_LINKS}
        ${LINK_LIBRARIES}
)


add_executable(
        orb_matching_pf
        nodes/orb_matching/orb_matching_pf.cpp
        nodes/orb_matching/Matcher.cpp
        nodes/orb_matching/FusionOdometry.cpp
        nodes/common.cpp
)

add_dependencies(orb_matching_pf ${catkin_EXPORTED_TARGETS})

target_link_libraries(
        orb_matching_pf
        ${ORB_BIN_LINKS}
        ${LINK_LIBRARIES}
)


add_executable(dumpmap
        nodes/dumpmap/dumpmap.cc
        )

target_link_libraries(dumpmap
        ${ORB_BIN_LINKS}
        ${LINK_LIBRARIES}
        )


add_executable(
        orb_mapping_offline
        nodes/orb_mapping/orb_mapping_offline.cpp
        nodes/common.cpp
)

target_link_libraries(
        orb_mapping_offline
        ${ORB_BIN_LINKS}
        ${LINK_LIBRARIES}
        boost_program_options
)


add_executable(
        map_publisher
        nodes/visualization/map_publisher.cc
        nodes/common.cpp
)

target_link_libraries(
        map_publisher
        ${catkin_LIBRARIES}
        ${ORB_BIN_LINKS}
        ${LINK_LIBRARIES}
)


add_executable(
        imgprocx
        nodes/imgprocx/imgprocx.cpp
        nodes/imgprocx/ImagePreprocessor.cpp
)

target_link_libraries(
        imgprocx
        ${catkin_LIBRARIES}
)


