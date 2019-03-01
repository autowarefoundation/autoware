cmake_minimum_required(VERSION 3.5)
project(range_vision_fusion)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(roscpp REQUIRED)
find_package(tf REQUIRED)
find_package(jsk_recognition_utils REQUIRED)
find_package(eigen_conversions REQUIRED)
find_package(tf_conversions REQUIRED)
find_package(jsk_topic_tools REQUIRED)
find_package(image_geometry REQUIRED)
find_package(jsk_topic_tools REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Eigen3 QUIET)

find_package(OpenGL REQUIRED)

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

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})

#fusion Library
add_library(range_vision_fusion_lib SHARED
        src/range_vision_fusion.cpp
        include/range_vision_fusion/range_vision_fusion.h
        )

if (OPENMP_FOUND)
    set_target_properties(range_vision_fusion_lib PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

target_include_directories(range_vision_fusion_lib PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${autoware_msgs_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        include
        )

target_link_libraries(range_vision_fusion_lib
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

ament_target_dependencies(range_vision_fusion_lib
        cv_bridge
        sensor_msgs
        image_transport
        roscpp
        tf
        autoware_msgs
        jsk_recognition_utils
        eigen_conversions
        tf_conversions
        image_geometry
        jsk_topic_tools
        )

#Fusion node
add_executable(range_vision_fusion
        src/range_vision_fusion_node.cpp
        )

target_include_directories(range_vision_fusion PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${autoware_msgs_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        include
        )

target_link_libraries(range_vision_fusion
        range_vision_fusion_lib)

install(TARGETS range_vision_fusion
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION include/${PROJECT_NAME}/launch/
        PATTERN ".svn" EXCLUDE)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(image_transport)
ament_export_dependencies(roscpp)
ament_export_dependencies(tf)
ament_export_dependencies(autoware_msgs)
ament_export_dependencies(jsk_recognition_utils)
ament_export_dependencies(eigen_conversions)
ament_export_dependencies(tf_conversions)
ament_export_dependencies(image_geometry)
ament_export_dependencies(jsk_topic_tools)

ament_export_include_directories(include)

ament_export_libraries(range_vision_fusion_lib)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()