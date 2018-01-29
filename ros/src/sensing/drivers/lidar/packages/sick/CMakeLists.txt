cmake_minimum_required(VERSION 2.8.3)
project(sick)

find_package(catkin REQUIRED COMPONENTS message_generation tf roscpp rospy std_msgs)

catkin_package(
  INCLUDE_DIRS nodes/lms511/include
  LIBRARIES sick
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
	${catkin_INCLUDE_DIRS} 
	${roscpp_INCLUDE_DIRS}
	nodes/lms511/include/lms511
)

add_executable(lms511_node nodes/lms511/src/lms5xx_node.cpp nodes/lms511/src/SickLMS5xx.cc nodes/lms511/src/SickLMS5xxBufferMonitor.cc nodes/lms511/src/SickLMS5xxMessage.cc)
add_dependencies(lms511_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lms511_node
   ${catkin_LIBRARIES}
)
