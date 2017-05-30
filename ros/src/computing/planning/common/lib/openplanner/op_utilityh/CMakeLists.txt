cmake_minimum_required(VERSION 2.8.3)
project(op_utilityh)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
find_package(catkin REQUIRED COMPONENTS
)

find_package(TinyXML REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES op_utilityh
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

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

