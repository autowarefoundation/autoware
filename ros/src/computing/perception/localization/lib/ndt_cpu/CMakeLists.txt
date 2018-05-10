cmake_minimum_required(VERSION 2.8.3)
project(ndt_cpu)

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)

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

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ndt_cpu                   #The exported libraries from the project
        DEPENDS PCL
)

include_directories(
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        "include"
        ${EIGEN3_INCLUDE_DIRS}
)

set(srcs
        src/NormalDistributionsTransform.cpp
        src/Registration.cpp
        src/VoxelGrid.cpp
        src/Octree.cpp
        )

set(incs
        include/ndt_cpu/debug.h
        include/ndt_cpu/NormalDistributionsTransform.h
        include/ndt_cpu/Registration.h
        include/ndt_cpu/SymmetricEigenSolver.h
        include/ndt_cpu/VoxelGrid.h
        include/ndt_cpu/Octree.h
        )

add_library(ndt_cpu ${incs} ${srcs})

target_link_libraries(ndt_cpu
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


install(TARGETS ndt_cpu
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )