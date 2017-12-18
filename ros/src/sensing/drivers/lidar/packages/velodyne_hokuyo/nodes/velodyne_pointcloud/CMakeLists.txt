cmake_minimum_required(VERSION 2.8.3)
project(vh_velodyne_pointcloud)

generate_dynamic_reconfigure_options(
  cfg/VelodyneConfig.cfg)

include_directories(include 
        ${catkin_INCLUDE_DIRS}
)

#catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES velodyne_rawdata)
    
add_subdirectory(src/lib)
add_subdirectory(src/conversions)

  
