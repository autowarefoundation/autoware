cmake_minimum_required(VERSION 2.8.3)
project(pcl_omp_registration)

find_package(catkin REQUIRED COMPONENTS)
find_package(PCL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES pcl_omp_registration
)

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

IF (PCL_VERSION VERSION_LESS "1.7.2")
    message("pcl_omp requires PCL 1.7.2 or higher versions")
ELSE (PCL_VERSION VERSION_LESS "1.7.2")


    set(srcs
            src/ndt.cpp
            )

    set(incs
            include/pcl_omp_registration/ndt.h
            include/pcl_omp_registration/registration.h
            )

    set(impl_incs
            include/pcl_omp_registration/impl/ndt.hpp
            include/pcl_omp_registration/impl/registration.hpp
            )

    include_directories(include
            ${PCL_INCLUDE_DIRS}
            ${EIGEN3_INCLUDE_DIRS}
            )

    add_library(pcl_omp_registration
            ${srcs}
            ${incs}
            ${impl_incs}
            )

    target_link_libraries(pcl_omp_registration
            ${PCL_LIBRARIES}
            ${EIGEN3_LIBRARIES}
            )

    install(DIRECTORY include/${PROJECT_NAME}/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
            )


    install(TARGETS pcl_omp_registration
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
ENDIF (PCL_VERSION VERSION_LESS "1.7.2")
