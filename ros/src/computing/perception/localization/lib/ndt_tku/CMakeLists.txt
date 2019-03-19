cmake_minimum_required(VERSION 2.8.3)
project(ndt_tku)

###################################
## catkin specific configuration ##
###################################
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ndt_tku
)

find_package(GLUT REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
)

add_library(ndt_tku
        src/algebra.cpp
        src/newton.cpp
        src/manage_ND.cpp
        )

target_link_libraries(ndt_tku
        ${catkin_LIBRARIES}
        ${GLUT_LIBRARIES})

install(TARGETS ndt_tku
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )