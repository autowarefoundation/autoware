cmake_minimum_required(VERSION 2.8.3)
project(op_utility)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(autoware_build_flags REQUIRED)

find_package(vector_map_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	vector_map_msgs 
	vector_map_server
)

find_package(TinyXML REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES op_utility
        CATKIN_DEPENDS vector_map_msgs vector_map_server
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS}
        include)

set(UTILITYH_SRC
        src/DataRW.cpp
        src/UtilityH.cpp
        )

## Declare a cpp library
add_library(${PROJECT_NAME}
        ${UTILITYH_SRC}
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${TinyXML_LIBRARIES}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


install(TARGETS op_utility
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test-op_utility
      test/test_op_utility.test
      test/src/test_op_utility.cpp
    )
    add_dependencies(test-op_utility ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-op_utility ${catkin_LIBRARIES} ${PROJECT_NAME})
endif ()
