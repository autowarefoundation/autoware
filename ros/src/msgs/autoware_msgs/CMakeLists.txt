cmake_minimum_required(VERSION 2.8.3)
project(autoware_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
        )


## Generate messages in the 'msg' folder
add_message_files(
        DIRECTORY msg
        FILES
            ControlCommandStamped.msg
            CloudCluster.msg
            CloudClusterArray.msg
            ColorSet.msg
            ControlCommand.msg
            DetectedObject.msg
            DetectedObjectArray.msg
            ExtractedPosition.msg
            ImageLaneObjects.msg
            ImageObjects.msg
            LaneArray.msg
            PointsImage.msg
            ScanImage.msg
            Signals.msg
            TunedResult.msg
            ValueSet.msg
            Centroids.msg
            DTLane.msg
            GeometricRectangle.msg
            ICPStat.msg
            ImageObj.msg
            ImageObjRanged.msg
            ImageObjTracked.msg
            ImageRect.msg
            ImageRectRanged.msg
            Lane.msg
            NDTStat.msg
            ObjLabel.msg
            ObjPose.msg
            ProjectionMatrix.msg
            VscanTracked.msg
            VscanTrackedArray.msg
            Waypoint.msg
            WaypointState.msg
            VehicleCmd.msg
            VehicleLocation.msg
            VehicleStatus.msg
            TrafficLightResult.msg
            TrafficLightResultArray.msg

            AccelCmd.msg
            AdjustXY.msg
            BrakeCmd.msg
            IndicatorCmd.msg
            LampCmd.msg
            SteerCmd.msg
            TrafficLight.msg
            StateCmd.msg
            State.msg
            ## Sync
            SyncTimeMonitor.msg
            SyncTimeDiff.msg

            ## Remote Control
            RemoteCmd.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        sensor_msgs
        jsk_recognition_msgs
)
