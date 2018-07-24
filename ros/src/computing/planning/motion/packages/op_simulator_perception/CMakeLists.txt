cmake_minimum_required(VERSION 2.8.3)
project(op_simulator_perception)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        geometry_msgs
        pcl_conversions
        pcl_ros
        op_utility
        op_planner
        autoware_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp geometry_msgs op_utility op_planner autoware_msgs
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


link_directories(lib)
add_executable(op_simulator_perception nodes/OpenPlannerSimulatorPerception.cpp nodes/OpenPlannerSimulatorPerception_core.cpp)
target_link_libraries(op_simulator_perception ${catkin_LIBRARIES})

add_dependencies(op_simulator_perception
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS op_simulator_perception
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )


install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
