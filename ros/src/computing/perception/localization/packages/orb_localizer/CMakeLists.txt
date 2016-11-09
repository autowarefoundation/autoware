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
find_package (GLEW REQUIRED)
find_package (BLAS REQUIRED)
find_package (LAPACK REQUIRED)
find_package (OpenMP)
find_package (Eigen3 3.1.0 REQUIRED)
find_package (PCL 1.7 REQUIRED COMPONENTS common octree)
find_package (PythonLibs REQUIRED)

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
include (Thirdparty/Pangolin/CMakeLists.txt)
