cmake_minimum_required(VERSION 2.8.3)
project (orb_localizer)


LIST(APPEND CMAKE_MODULE_PATH ${orb_localizer_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS 
	tf 
	sensor_msgs
	image_transport
	rosbag
	cv_bridge
	message_generation
	std_msgs
)

find_package(Boost REQUIRED COMPONENTS system python)
find_package (OpenCV REQUIRED)
find_package (FREEGLUT REQUIRED)
find_package (GLEW REQUIRED)
FIND_PACKAGE(BLAS REQUIRED)
FIND_PACKAGE(LAPACK REQUIRED)
FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND G2O_USE_OPENMP)
  SET (G2O_OPENMP 1)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)
SET(EIGEN3_INCLUDE_DIR ${G2O_EIGEN3_INCLUDE})
FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
IF(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
ENDIF(EIGEN3_FOUND)
find_package(PCL 1.7 REQUIRED COMPONENTS common octree)
#find_package(Pangolin REQUIRED)
find_package(PythonLibs REQUIRED)

# For building Pangolin
include (Thirdparty/Pangolin/CMakeLists.txt)

# Debug
find_package(message_generation)


# Force using C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")



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
)

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

# G2O
	Thirdparty/g2o/core
	Thirdparty/g2o/types
	Thirdparty/g2o/stuff 
	${G2O_EIGEN3_INCLUDE}

# DBoW2
	Thirdparty
	Thirdparty/DBoW2

# Pangolin
	Thirdparty/Pangolin/include

)


add_library(DBoW2 STATIC 
	Thirdparty/DBoW2/BowVector.cpp
	Thirdparty/DBoW2/FORB.cpp      
	Thirdparty/DBoW2/FeatureVector.cpp
	Thirdparty/DBoW2/ScoringObject.cpp
	Thirdparty/DBoW2/DUtils/Random.cpp
	Thirdparty/DBoW2/DUtils/Timestamp.cpp
)

# Building g2o
configure_file(
	${orb_localizer_SOURCE_DIR}/Thirdparty/g2o/config.h.in 
	${orb_localizer_SOURCE_DIR}/Thirdparty/g2o/config.h
)

add_library (g2o STATIC
	#types
	Thirdparty/g2o/types/types_sba.h
	Thirdparty/g2o/types/types_six_dof_expmap.h
	Thirdparty/g2o/types/types_sba.cpp
	Thirdparty/g2o/types/types_six_dof_expmap.cpp
	Thirdparty/g2o/types/types_seven_dof_expmap.cpp
	Thirdparty/g2o/types/types_seven_dof_expmap.h
	Thirdparty/g2o/types/se3quat.h
	Thirdparty/g2o/types/se3_ops.h
	Thirdparty/g2o/types/se3_ops.hpp
	#core
	Thirdparty/g2o/core/base_edge.h
	Thirdparty/g2o/core/base_binary_edge.h
	Thirdparty/g2o/core/hyper_graph_action.cpp
	Thirdparty/g2o/core/base_binary_edge.hpp
	Thirdparty/g2o/core/hyper_graph_action.h
	Thirdparty/g2o/core/base_multi_edge.h           
	Thirdparty/g2o/core/hyper_graph.cpp
	Thirdparty/g2o/core/base_multi_edge.hpp         
	Thirdparty/g2o/core/hyper_graph.h
	Thirdparty/g2o/core/base_unary_edge.h          
	Thirdparty/g2o/core/linear_solver.h
	Thirdparty/g2o/core/base_unary_edge.hpp         
	Thirdparty/g2o/core/marginal_covariance_cholesky.cpp
	Thirdparty/g2o/core/base_vertex.h               
	Thirdparty/g2o/core/marginal_covariance_cholesky.h
	Thirdparty/g2o/core/base_vertex.hpp             
	Thirdparty/g2o/core/matrix_structure.cpp
	Thirdparty/g2o/core/batch_stats.cpp             
	Thirdparty/g2o/core/matrix_structure.h
	Thirdparty/g2o/core/batch_stats.h               
	Thirdparty/g2o/core/openmp_mutex.h
	Thirdparty/g2o/core/block_solver.h              
	Thirdparty/g2o/core/block_solver.hpp            
	Thirdparty/g2o/core/parameter.cpp               
	Thirdparty/g2o/core/parameter.h                 
	Thirdparty/g2o/core/cache.cpp                   
	Thirdparty/g2o/core/cache.h
	Thirdparty/g2o/core/optimizable_graph.cpp       
	Thirdparty/g2o/core/optimizable_graph.h         
	Thirdparty/g2o/core/solver.cpp                  
	Thirdparty/g2o/core/solver.h
	Thirdparty/g2o/core/creators.h                 
	Thirdparty/g2o/core/optimization_algorithm_factory.cpp
	Thirdparty/g2o/core/estimate_propagator.cpp     
	Thirdparty/g2o/core/optimization_algorithm_factory.h
	Thirdparty/g2o/core/estimate_propagator.h       
	Thirdparty/g2o/core/factory.cpp                 
	Thirdparty/g2o/core/optimization_algorithm_property.h
	Thirdparty/g2o/core/factory.h                   
	Thirdparty/g2o/core/sparse_block_matrix.h
	Thirdparty/g2o/core/sparse_optimizer.cpp  
	Thirdparty/g2o/core/sparse_block_matrix.hpp
	Thirdparty/g2o/core/sparse_optimizer.h
	Thirdparty/g2o/core/hyper_dijkstra.cpp 
	Thirdparty/g2o/core/hyper_dijkstra.h
	Thirdparty/g2o/core/parameter_container.cpp     
	Thirdparty/g2o/core/parameter_container.h
	Thirdparty/g2o/core/optimization_algorithm.cpp 
	Thirdparty/g2o/core/optimization_algorithm.h
	Thirdparty/g2o/core/optimization_algorithm_with_hessian.cpp 
	Thirdparty/g2o/core/optimization_algorithm_with_hessian.h
	Thirdparty/g2o/core/optimization_algorithm_levenberg.cpp 
	Thirdparty/g2o/core/optimization_algorithm_levenberg.h
	Thirdparty/g2o/core/jacobian_workspace.cpp 
	Thirdparty/g2o/core/jacobian_workspace.h
	Thirdparty/g2o/core/robust_kernel.cpp 
	Thirdparty/g2o/core/robust_kernel.h
	Thirdparty/g2o/core/robust_kernel_factory.cpp
	Thirdparty/g2o/core/robust_kernel_factory.h
	Thirdparty/g2o/core/robust_kernel_impl.cpp 
	Thirdparty/g2o/core/robust_kernel_impl.h
	#stuff
	Thirdparty/g2o/stuff/string_tools.h
	Thirdparty/g2o/stuff/color_macros.h 
	Thirdparty/g2o/stuff/macros.h
	Thirdparty/g2o/stuff/timeutil.cpp
	Thirdparty/g2o/stuff/misc.h
	Thirdparty/g2o/stuff/timeutil.h
	Thirdparty/g2o/stuff/os_specific.c    
	Thirdparty/g2o/stuff/os_specific.h
	Thirdparty/g2o/stuff/string_tools.cpp
	Thirdparty/g2o/stuff/property.cpp       
	Thirdparty/g2o/stuff/property.h       
)

add_library(
	orb_slam2 SHARED
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

target_link_libraries(
	orb_slam2
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
#	${Pangolin_LIBRARIES}
	pangolin
	DBoW2
	g2o
	boost_system
	boost_serialization
	${PCL_LIBRARIES}
	glut
	GLEW
)


set (orb_slam_vocabulary_file ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/ORBvoc.txt)
add_custom_target (orb_vocabulary ALL
	[ ! -e ${orb_slam_vocabulary_file} ] && tar -zxf ${PROJECT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz --directory ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME} || return 0
	DEPENDS Vocabulary/ORBvoc.txt.tar.gz
)
add_definitions ( 
	-DORB_SLAM_VOCABULARY="${orb_slam_vocabulary_file}"
)


add_executable (orb_mapping
	src/nodes/mapper.cc
	src/nodes/utils.cc
	src/nodes/ImageGrabber.cc
)
add_dependencies (orb_mapping 
	orb_vocabulary 
	orb_localizer_generate_messages_cpp)

add_custom_target (orb_matching ALL
	cd ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME} && ln -sf orb_mapping orb_matching
)
add_dependencies (orb_matching 
	orb_mapping
	orb_localizer_generate_messages_cpp
)

target_link_libraries (orb_mapping
	${PCL_LIBRARIES}
	${LIBS}
	orb_slam2
	${catkin_LIBRARIES}
)

add_executable (dumpmap
	src/nodes/dumpmap.cc
)

target_link_libraries (dumpmap
	orb_slam2
)

add_executable (map_publisher
	src/nodes/map_publisher.cc
	src/nodes/ImageGrabber.cc
	src/nodes/utils.cc
)

target_link_libraries (map_publisher
	orb_slam2
	${catkin_LIBRARIES}
)

#add_executable (bag_mapper
#	src/nodes/bag_mapper.cc
#	src/nodes/ImageGrabber.cc
#	src/nodes/utils.cc
#)
#target_compile_definitions(bag_mapper PUBLIC ORB_SLAM_VOCABULARY="${orb_slam_vocabulary_file}")
 
#target_link_libraries (bag_mapper
#	${PCL_LIBRARIES}
#	${LIBS}
#	orb_slam2
#	${catkin_LIBRARIES}
#)

#add_dependencies (bag_mapper
#	orb_localizer_generate_messages_cpp
#)

#add_executable (offline_offset
#	src/nodes/offline_offset.cc
#	src/nodes/ImageGrabber.cc
#	src/nodes/utils.cc
#)

#target_link_libraries (offline_offset
#	${PCL_LIBRARIES}
#	${LIBS}
#	orb_slam2
#	${catkin_LIBRARIES}
#)


add_library (_orb_slam2 SHARED
	src/analysis/orbslam2_python.cc
)
target_link_libraries (_orb_slam2
	boost_python
	orb_slam2
)
set_target_properties(_orb_slam2 PROPERTIES
	PREFIX ""
)
add_dependencies (_orb_slam2 orb_slam2)


#add_executable (orb_matching_mt
#	src/nodes/mt/matching_mt.cc
#	src/nodes/mt/TrackingThread.cc
#	src/nodes/mt/SystemMT.cc
#	src/nodes/mt/DebugMT.cc
#	src/nodes/ImageGrabber.cc
#	src/nodes/utils.cc
#)
#target_link_libraries (orb_matching_mt
#	orb_slam2
#	${catkin_LIBRARIES}
#	${PCL_LIBRARIES}
#)
#add_dependencies (orb_matching_mt
#	orb_vocabulary
#	orb_localizer_generate_messages_cpp
#)

