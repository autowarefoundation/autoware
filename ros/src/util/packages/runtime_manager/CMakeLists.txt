cmake_minimum_required(VERSION 2.8.3)
project(runtime_manager)

find_package(catkin REQUIRED COMPONENTS
        rospy
        )

catkin_package(
        CATKIN_DEPENDS rospy std_msgs message_runtime
)

