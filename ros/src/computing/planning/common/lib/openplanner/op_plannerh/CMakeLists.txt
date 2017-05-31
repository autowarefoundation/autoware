cmake_minimum_required(VERSION 2.8.3)
project(op_plannerh)

# Get a ros version
execute_process(
	COMMAND rosversion -d
	OUTPUT_VARIABLE ROS_VERSION
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(catkin REQUIRED COMPONENTS
		op_utilityh
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES  op_plannerh
   CATKIN_DEPENDS op_utilityh
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")

include_directories(${catkin_INCLUDE_DIRS}
		include
)

set(PLANNERH_SRC
		src/BehaviorStateMachine.cpp
		src/MappingHelpers.cpp
		src/RSPlanner.cpp
		src/GridMap.cpp
		src/MatrixOperations.cpp
		src/TrajectoryCosts.cpp
		src/HMIStateMachine.cpp
		src/PlannerH.cpp
		src/geo_pos_conv.cpp
		src/LocalPlannerH.cpp
		src/PlanningHelpers.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME}
	    ${PLANNERH_SRC}
)

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
)

