cmake_minimum_required(VERSION 3.5)
project(runtime_manager)

find_package(catkin REQUIRED COMPONENTS
        rospy
        )

catkin_package(
        CATKIN_DEPENDS rospy std_msgs message_runtime
)

