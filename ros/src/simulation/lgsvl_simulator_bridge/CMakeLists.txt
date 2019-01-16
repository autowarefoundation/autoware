cmake_minimum_required(VERSION 2.8.3)
project(lgsvl_simulator_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        autoware_msgs
        geometry_msgs
        roscpp
        rospy
        roslib
        sensor_msgs
        rqt_gui
        rqt_gui_py
)

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS
        autoware_msgs
        geometry_msgs
        roscpp
        rospy
        sensor_msgs
        rqt_gui
        rqt_gui_py
)


include_directories(
        ${catkin_INCLUDE_DIRS}
)

foreach (programs)
    install(TARGETS ${programs}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
endforeach (programs)

foreach (dir launch rviz)
    install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach (dir)

install(PROGRAMS scripts/launch_sim.bash src/lgsvl_simulator_bridge/__init__.py src/lgsvl_simulator_bridge/configurator.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )