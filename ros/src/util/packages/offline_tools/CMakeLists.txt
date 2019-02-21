cmake_minimum_required(VERSION 2.8)
project(offline_tools)

add_compile_options(-std=c++11)

set (CMAKE_INCLUDE_CURRENT_DIR		ON)
set (CMAKE_AUTOMOC					ON)
set (CMAKE_AUTOUIC					ON)


find_package(catkin REQUIRED COMPONENTS 
	rviz
	cv_bridge
	pcl_ros
	rosbag_storage)

#find_package(Qt5 REQUIRED Core Widgets)
find_package (Qt5Core CONFIG REQUIRED)
find_package (Qt5Widgets CONFIG REQUIRED)
find_package(PCL REQUIRED)
find_package (OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  	rviz_offline_display
  	random_access_bag
  CATKIN_DEPENDS 
	roscpp 
	rviz 
	rosbag_storage
)


include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	common
	include
)

link_directories(
	${catkin_LIBRARY_DIRS})


add_library(
	rviz_offline_display
		rviz_plugins/PCDFileDisplay.cpp
		rviz_plugins/CSVTrajectory.cpp
		rviz_plugins/FileProperty.cpp
)

target_link_libraries(
	rviz_offline_display
		Qt5::Core
		Qt5::Widgets
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
)

add_library(
	random_access_bag
		random_access_bag/RandomAccessBag.cpp
)

target_link_libraries(
	random_access_bag
		${catkin_LIBRARIES}
)


add_executable(bag_viewer
	bag_viewer/main.cpp
	bag_viewer/BagViewer.cpp
	bag_viewer/ClickableLabel.cpp
	bag_viewer/ratio_layouted_frame.cpp
	bag_viewer/BagViewer.ui
	bag_viewer/ratio_layouted_frame.h
	bag_viewer/ClickableLabel.h
)

target_link_libraries(
	bag_viewer
		random_access_bag
		${catkin_LIBRARIES}
		Qt5::Widgets
		${OpenCV_LIBS}
)

target_include_directories(bag_viewer
	PUBLIC bag_viewer
)


if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest_gtest(bag_test 
		test/test_random_access_bag.launch 
		test/test_random_access_bag.cpp)
	target_link_libraries(bag_test 
		random_access_bag 
		${catkin_LIBRARIES})
endif ()

