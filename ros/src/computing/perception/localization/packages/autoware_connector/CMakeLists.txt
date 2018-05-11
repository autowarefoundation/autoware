cmake_minimum_required(VERSION 2.8.3)
project(autoware_connector)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        geometry_msgs
        autoware_msgs
        nav_msgs
        std_msgs
        )


catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(can_status_translator
        nodes/can_status_translator/can_status_translator_node.cpp
        nodes/can_status_translator/can_status_translator_core.cpp
        )

target_link_libraries(can_status_translator
        ${catkin_LIBRARIES}
        )

add_dependencies(can_status_translator
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(can_odometry
        nodes/can_odometry/can_odometry_node.cpp
        nodes/can_odometry/can_odometry_core.cpp
        )

target_link_libraries(can_odometry
        ${catkin_LIBRARIES}
        )

add_dependencies(can_odometry
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS can_status_translator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS can_odometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
