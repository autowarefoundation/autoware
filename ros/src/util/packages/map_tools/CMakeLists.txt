cmake_minimum_required(VERSION 2.8.3)
project(map_tools)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        tf
        sensor_msgs
        pcl_ros
        pcl_conversions
        )

catkin_package(
        CATKIN_DEPENDS roscpp
        tf
        sensor_msgs
        pcl_ros
        pcl_conversions
        DEPENDS PCL
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pcd_filter nodes/pcd_filter/pcd_filter.cpp)
add_executable(pcd_binarizer nodes/pcd_binarizer/pcd_binarizer.cpp)
add_executable(pcd_arealist nodes/pcd_arealist/pcd_arealist.cpp)
add_executable(csv2pcd nodes/pcd_converter/csv2pcd.cpp)
add_executable(pcd2csv nodes/pcd_converter/pcd2csv.cpp)
add_executable(map_extender nodes/map_extender/map_extender.cpp)
add_executable(pcd_grid_divider nodes/pcd_grid_divider/pcd_grid_divider.cpp)

target_link_libraries(pcd_filter ${catkin_LIBRARIES})
target_link_libraries(pcd_binarizer ${catkin_LIBRARIES})
target_link_libraries(pcd_arealist ${catkin_LIBRARIES})
target_link_libraries(csv2pcd ${catkin_LIBRARIES})
target_link_libraries(pcd2csv ${catkin_LIBRARIES})
target_link_libraries(map_extender ${catkin_LIBRARIES})
target_link_libraries(pcd_grid_divider ${catkin_LIBRARIES})


install(TARGETS pcd_filter pcd_binarizer pcd_arealist csv2pcd pcd2csv map_extender pcd_grid_divider
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
