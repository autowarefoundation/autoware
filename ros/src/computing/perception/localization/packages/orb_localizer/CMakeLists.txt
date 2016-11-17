cmake_minimum_required (VERSION 2.8.3)
project (orb_localizer)

find_package (catkin REQUIRED
	COMPONENTS
		roscpp
		tf 
		sensor_msgs
		image_transport
		cv_bridge
		message_generation
		std_msgs
)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_NAME_SOURCE_DIR}/cmake_modules)

# Force using C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

find_package (Boost REQUIRED COMPONENTS system python)
find_package (OpenCV REQUIRED)
find_package (OpenGL REQUIRED)
find_package (GLEW REQUIRED)
find_package (X11 REQUIRED)
find_package (BLAS REQUIRED)
find_package (LAPACK REQUIRED)
find_package (OpenMP)
find_package (Eigen3 3.1.0 REQUIRED)
find_package (PCL 1.7 REQUIRED COMPONENTS common octree)
find_package (PythonLibs REQUIRED)


add_message_files (
	FILES
		debug.msg
)

generate_messages (
	DEPENDENCIES
		std_msgs
)


catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
	DEPENDS OpenCV GLEW PCL
)


# Vocabulary file
set (orb_slam_vocabulary_file ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/ORBvoc.txt)
add_custom_target (orb_vocabulary ALL
	[ ! -e ${orb_slam_vocabulary_file} ] && tar -zxf ${PROJECT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz --directory ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME} || return 0
	DEPENDS Vocabulary/ORBvoc.txt.tar.gz
)
add_definitions ( 
	-DORB_SLAM_VOCABULARY="${orb_slam_vocabulary_file}"
)

# Configuration file
configure_file (
	config/orb-slam2.yaml.in
	${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/orb-slam2.yaml
)

# For building Pangolin
add_subdirectory (Thirdparty/Pangolin)

# For building DBoW2
add_subdirectory (Thirdparty/DBoW2)

# For building g2o
add_subdirectory (Thirdparty/g2o)


include_directories (
	# my own
	include
	${catkin_INCLUDE_DIRS}
	
# Important libraries
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIR}
	
# Third parties
	Thirdparty
	Thirdparty/g2o
	Thirdparty/Pangolin/include
	
)


add_library(
	orb_slam2 STATIC
		src/System.cc
		src/Tracking.cc
		src/LocalMapping.cc
		src/LoopClosing.cc
		src/ORBextractor.cc
		src/ORBmatcher.cc
		src/FrameDrawer.cc
		src/Converter.cc
		src/MapPoint.cc
		src/KeyFrame.cc
		src/Map.cc
		src/MapDrawer.cc
		src/Optimizer.cc
		src/PnPsolver.cc
		src/Frame.cc
		src/KeyFrameDatabase.cc
		src/Sim3Solver.cc
		src/Initializer.cc
		src/Viewer.cc
)


list (APPEND LINK_LIBRARIES
	boost_system
	boost_serialization
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${X11_LIBRARIES}
	${OPENGL_LIBRARIES}
	${GLEW_LIBRARIES}
	${PCL_LIBRARIES}
)

list (APPEND ORB_BIN_LINKS
	orb_slam2
	DBoW2
	g2o
	pangolin
	${LINK_LIBRARIES}
)


# Main Executables
add_executable (
	orb_mapping
		nodes/orb_mapping/orb_mapping.cpp
		nodes/common.cpp
)

target_link_libraries (
	orb_mapping
		${ORB_BIN_LINKS}		
)

add_executable (
	orb_matching
		nodes/orb_matching/orb_matching.cpp
		nodes/common.cpp
)

target_link_libraries (
	orb_matching
		${ORB_BIN_LINKS}
)

add_executable (dumpmap
	nodes/dumpmap/dumpmap.cc
)

target_link_libraries (dumpmap
	${ORB_BIN_LINKS}
)
