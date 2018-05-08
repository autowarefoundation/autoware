cmake_minimum_required(VERSION 2.8.3)
project(adi_driver)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs roslint)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(adis16470_node src/adis16470_node.cpp src/adis16470.cpp)
target_link_libraries(adis16470_node ${catkin_LIBRARIES})

add_executable(adxl345_node src/adxl345_node.cpp src/adxl345.cpp)
target_link_libraries(adxl345_node ${catkin_LIBRARIES})

install(TARGETS adis16470_node adxl345_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

install(DIRECTORY launch test urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

#if(CATKIN_ENABLE_TESTING)
#  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
#  file(GLOB LAUNCH_FILES launch/*.launch)
#  foreach(LAUNCH_FILE ${LAUNCH_FILES})
#    message(STATUS "Testing ${LAUNCH_FILE}")
#    roslaunch_add_file_check(${LAUNCH_FILE})
#  endforeach()
#  roslaunch_add_file_check(launch/adis16470.launch with_rviz:=true with_plot:=true)
#endif()

roslint_cpp()

# catkin_add_gtest(${PROJECT_NAME}-test test/test_adi_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# debbuild
# catkin build minas_control --make-args debbuild
# ROS_PARALLEL_JOBS='' catkin_make --pkg ${PROJECT_NAME} -- debbuild
add_custom_target(debbuild_${PROJECT_NAME}
  COMMAND sudo -n true || (>&2 echo "debbuild need sudo access to make debs" && exit 1)
  COMMAND bloom-generate rosdebian --os-name ubuntu --os-version trusty --ros-distro indigo
  COMMAND MAKEFLAGS= fakeroot debian/rules binary
  COMMAND find debian obj-x86_64-linux-gnu -type f -not -name postinst -print0 | xargs -0 rm --
  COMMAND find debian obj-x86_64-linux-gnu -type d -empty -delete
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )

# make docs
# ROS_PARALLEL_JOBS='' catkin_make --pkg adi_driver -- docbuild_adi_driver
add_custom_target(docbuild_${PROJECT_NAME}
  COMMAND cmake -E remove_directory output
  COMMAND cmake -E make_directory output
  COMMAND sphinx-build -b latex doc output
  #COMMAND cmake -E chdir output sh -c 'ls *.png | xargs extractbb'
  COMMAND cd output \\; ls *.png | xargs extractbb
  COMMAND sed -i 's@\\\\setcounter{tocdepth}{0}@@' output/${PROJECT_NAME}.tex
  ## change layout, chapter to section, section to subsection
    COMMAND sed -i 's@\\\\chapter{[0-9. ]*@\\\\chapter{@' output/${PROJECT_NAME}.tex
    COMMAND sed -i 's@\\\\section{[0-9. ]*@\\\\section{@' output/${PROJECT_NAME}.tex
    COMMAND sed -i 's@\\\\subsection@\\\\subsubsection@' output/${PROJECT_NAME}.tex
    COMMAND sed -i 's@\\\\section@\\\\subsection@' output/${PROJECT_NAME}.tex
    COMMAND sed -i 's@\\\\chapter@\\\\section@' output/${PROJECT_NAME}.tex
    COMMAND sed -i 's@\\\\tableofcontents@\\\\setcounter{chapter}{1}\\\\tableofcontents@' output/${PROJECT_NAME}.tex
  ##
  ## for japanese doc,
   # COMMAND cmake -E chdir output platex ${PROJECT_NAME}.tex
   # COMMAND cmake -E chdir output platex ${PROJECT_NAME}.tex
   # COMMAND cmake -E chdir output dvipdfmx ${PROJECT_NAME}
  ## for english doc
    COMMAND make -C output
  ##
  COMMAND cmake -E copy output/${PROJECT_NAME}.pdf output/${PROJECT_NAME}-${${PROJECT_NAME}_VERSION}.pdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )
