cmake_minimum_required(VERSION 2.8.3)
project(op_planner)

# Get a ros version
set(ROS_VERSION $ENV{ROS_DISTRO})

find_package(catkin REQUIRED COMPONENTS
		autoware_build_flags
		cmake_modules
		op_utility
)

find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES  op_planner
   CATKIN_DEPENDS op_utility
   DEPENDS TinyXML
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS}
		include
		${TinyXML_INCLUDE_DIRS}
)

set(PLANNERH_SRC
		src/BehaviorPrediction.cpp 
		src/BehaviorStateMachine.cpp
		src/DecisionMaker.cpp
		src/LocalPlannerH.cpp
		src/MappingHelpers.cpp
		src/MatrixOperations.cpp
		src/PassiveDecisionMaker.cpp
		src/PlannerH.cpp		
		src/PlanningHelpers.cpp				
		src/SimuDecisionMaker.cpp
		src/TrajectoryCosts.cpp
		src/TrajectoryDynamicCosts.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME}
	    ${PLANNERH_SRC}
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


install(TARGETS op_planner
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)
