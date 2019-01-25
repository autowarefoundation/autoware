find_package(vector_map_msgs REQUIRED)

add_library(velodyne_rawdata rawdata.cc calibration.cc)

add_dependencies(velodyne_rawdata
  ${vector_map_msgs_EXPORTED_TARGETS}
)
target_link_libraries(velodyne_rawdata 
                      ${catkin_LIBRARIES}
                      ${YAML_CPP_LIBRARIES})
install(TARGETS velodyne_rawdata
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
