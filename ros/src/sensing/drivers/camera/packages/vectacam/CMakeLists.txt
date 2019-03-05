cmake_minimum_required(VERSION 2.8.3)
project(vectacam)


find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        tf
        image_transport
        cv_bridge
        )

FIND_PACKAGE(OpenCV REQUIRED)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

catkin_package(
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

add_executable(vectacam_node
        nodes/vectacam/vectacam_node.cpp
        nodes/vectacam/VectaCam.cpp)

link_directories(${catkin_LIBRARY_DIRS}
        ${Boost_LIBRARY_DIRS})

target_link_libraries(vectacam_node
        ${BOOST_LIBRARIES}
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        -lpthread
        )

install(TARGETS
        vectacam_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)