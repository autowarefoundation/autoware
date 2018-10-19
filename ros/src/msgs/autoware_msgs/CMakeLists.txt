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
            CanInfo.msg
            CANData.msg
            CANPacket.msg
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
            VehicleStatus.msg
            TrafficLightResult.msg
            TrafficLightResultArray.msg

            ## Runtime Manager ##
            config/ConfigApproximateNdtMapping.msg
            config/ConfigCarDpm.msg
            config/ConfigCarFusion.msg
            config/ConfigCarKf.msg
            config/ConfigCompareMapFilter.msg
            config/ConfigDecisionMaker.msg
            config/ConfigDistanceFilter.msg
            config/ConfigRingGroundFilter.msg
            config/ConfigICP.msg
            config/ConfigLaneRule.msg
            config/ConfigLaneSelect.msg
            config/ConfigLaneStop.msg
            config/ConfigLatticeVelocitySet.msg
            config/ConfigNdt.msg
            config/ConfigNdtMapping.msg
            config/ConfigNdtMappingOutput.msg
            config/ConfigPedestrianDpm.msg
            config/ConfigPedestrianFusion.msg
            config/ConfigPedestrianKf.msg
            config/ConfigPlannerSelector.msg
            config/ConfigPoints2Polygon.msg
            config/ConfigRandomFilter.msg
            config/ConfigRingFilter.msg
            config/ConfigRayGroundFilter.msg
            config/ConfigSsd.msg
            config/ConfigTwistFilter.msg
            config/ConfigVelocitySet.msg
            config/ConfigVoxelGridFilter.msg
            config/ConfigWaypointLoader.msg
            config/ConfigWaypointFollower.msg
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
