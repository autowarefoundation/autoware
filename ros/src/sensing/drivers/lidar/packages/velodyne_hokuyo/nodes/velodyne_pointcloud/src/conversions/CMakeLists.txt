add_executable(vh_cloud_node cloud_node.cc convert.cc)
target_link_libraries(vh_cloud_node vh_velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS vh_cloud_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(vh_cloud_nodelet cloud_nodelet.cc convert.cc)
target_link_libraries(vh_cloud_nodelet vh_velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS vh_cloud_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(vh_ringcolors_node ringcolors_node.cc colors.cc)
target_link_libraries(vh_ringcolors_node
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS vh_ringcolors_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(vh_ringcolors_nodelet ringcolors_nodelet.cc colors.cc)
target_link_libraries(vh_ringcolors_nodelet
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS vh_ringcolors_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(vh_transform_node transform_node.cc transform.cc)
target_link_libraries(vh_transform_node vh_velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS vh_transform_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(vh_transform_nodelet transform_nodelet.cc transform.cc)
target_link_libraries(vh_transform_nodelet vh_velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS vh_transform_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
