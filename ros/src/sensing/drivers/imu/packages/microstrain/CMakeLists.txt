cmake_minimum_required(VERSION 2.8.3)
project(microstrain_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp
    cmake_modules
    tf2
    tf2_ros
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

###########
## Build ##
###########

# Directories containing MIP SDK header files
set(MIPLIBINC MIPSDK/C/Library/Include)
set(MIPLIBSRC MIPSDK/C/Library/Source)
set(MIPUTILINC MIPSDK/C/Utilities/Include)
set(MIPUTILSRC MIPSDK/C/Utilities/Source)

# The SDK comes with a skeleton for the user-defined functions
# It might be helpful to test compilation against these skeletons
# contains both .c and .h files
# set(MIPUSR "MIPSDK/C/Library/User\ Functions")
#include_directories(include ${MIPLIBINC} ${MIPUTILINC} ${MIPUSR})
include_directories(include/${PROJECT_NAME} ${MIPLIBINC} ${MIPUTILINC} ${catkin_INCLUDE_DIRS})

set(MIPINC
  ${MIPLIBINC}/mip.h
  ${MIPLIBINC}/mip_sdk_base.h    
  ${MIPLIBINC}/mip_sdk_inteface.h
  ${MIPLIBINC}/mip_sdk_3dm.h   
  ${MIPLIBINC}/mip_sdk_filter.h  
  ${MIPLIBINC}/mip_sdk_system.h
  ${MIPLIBINC}/mip_sdk_ahrs.h  
  ${MIPLIBINC}/mip_sdk_gps.h     
  ${MIPLIBINC}/ring_buffer.h
#  ${MIPUSR}/mip_sdk_user_functions.h
  ${MIPUTILINC}/byteswap_utilities.h
)

set(MIPSRC 
  ${MIPLIBSRC}/mip.c
  ${MIPLIBSRC}/mip_sdk_base.c    
  ${MIPLIBSRC}/mip_sdk_inteface.c
  ${MIPLIBSRC}/mip_sdk_3dm.c   
  ${MIPLIBSRC}/mip_sdk_filter.c  
  ${MIPLIBSRC}/mip_sdk_system.c
  ${MIPLIBSRC}/mip_sdk_ahrs.c  
  ${MIPLIBSRC}/mip_sdk_gps.c     
  ${MIPLIBSRC}/ring_buffer.c
#  ${MIPUSR}/mip_sdk_user_functions.c  
  ${MIPUTILSRC}/byteswap_utilities.c
)

# Libraries
add_library(microstrain_driver src/microstrain.cpp src/mip_sdk_user_functions.c ${MIPSRC})

# Executables
add_executable(3dm_gx5_15_node src/3dm_gx5_15_node.cpp)
add_executable(3dm_gx5_45_node src/3dm_gx5_45_node.cpp)

# Linking
target_link_libraries(microstrain_driver ${catkin_LIBRARIES})
target_link_libraries(3dm_gx5_15_node microstrain_driver ${catkin_LIBRARIES})
target_link_libraries(3dm_gx5_45_node microstrain_driver ${catkin_LIBRARIES})

install(TARGETS
        microstrain_driver
        3dm_gx5_15_node
        3dm_gx5_45_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)