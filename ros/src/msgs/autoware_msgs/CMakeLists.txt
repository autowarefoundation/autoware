cmake_minimum_required(VERSION 2.8.3)
project(autoware_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
        autoware_planning_msgs
        )


## Generate messages in the 'msg' folder
add_message_files(
        DIRECTORY msg
        FILES
            
            
            
            
            
            
            
            
            
            ImageObjects.msg
            
            
            ScanImage.msg
            
            
            
            
            
            GeometricRectangle.msg
            
            
            
            
            
            
            
            
            
            ObjPose.msg
            
            VscanTracked.msg
            VscanTrackedArray.msg
            
            
            
            
            
            

            
            
            
            IndicatorCmd.msg
            
            
            
            StateCmd.msg
            
            ## Sync
            
            

            ## Remote Control
            
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
        autoware_planning_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
        autoware_planning_msgs
)
