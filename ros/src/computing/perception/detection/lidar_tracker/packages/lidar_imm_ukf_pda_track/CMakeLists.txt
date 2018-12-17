cmake_minimum_required(VERSION 2.8.3)
project(imm_ukf_pda_track)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  pcl_ros
  autoware_msgs
  tf
  vector_map
  )


set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  autoware_msgs
  tf
  vector_map
  )

include_directories(
        ${catkin_INCLUDE_DIRS}
        include)

link_directories(${PCL_LIBRARY_DIRS})

#imm_ukf_pda
add_library(imm_ukf_pda_lib
        nodes/imm_ukf_pda/imm_ukf_pda_main.cpp
        nodes/imm_ukf_pda/imm_ukf_pda.cpp
        nodes/imm_ukf_pda/ukf.cpp
        )
target_link_libraries(imm_ukf_pda_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
add_dependencies(imm_ukf_pda_lib
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(imm_ukf_pda
        nodes/imm_ukf_pda/imm_ukf_pda_main.cpp
        nodes/imm_ukf_pda/imm_ukf_pda.cpp
        nodes/imm_ukf_pda/ukf.cpp
        )
target_link_libraries(imm_ukf_pda
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
add_dependencies(imm_ukf_pda
        ${catkin_EXPORTED_TARGETS}
        )


install(TARGETS
        imm_ukf_pda
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_imm_ukf_pda test/src/test_imm_ukf_pda.cpp)
  target_link_libraries(test_imm_ukf_pda ${catkin_LIBRARIES} imm_ukf_pda_lib)
endif()
