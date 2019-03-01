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
            
            CloudCluster.msg
            CloudClusterArray.msg
            ColorSet.msg
            
            DetectedObject.msg
            DetectedObjectArray.msg
            
            ImageLaneObjects.msg
            ImageObjects.msg
            
            
            ScanImage.msg
            Signals.msg
            TunedResult.msg
            ValueSet.msg
            Centroids.msg
            
            GeometricRectangle.msg
            
            ImageObj.msg
            ImageObjRanged.msg
            ImageObjTracked.msg
            ImageRect.msg
            ImageRectRanged.msg
            
            
            ObjLabel.msg
            ObjPose.msg
            
            VscanTracked.msg
            VscanTrackedArray.msg
            
            
            
            
            TrafficLightResult.msg
            TrafficLightResultArray.msg

            
            AdjustXY.msg
            
            IndicatorCmd.msg
            
            
            TrafficLight.msg
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
