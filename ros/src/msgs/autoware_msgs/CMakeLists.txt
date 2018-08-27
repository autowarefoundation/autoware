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
            centroids.msg
            dtlane.msg
            geometric_rectangle.msg
            icp_stat.msg
            image_obj.msg
            image_obj_ranged.msg
            image_obj_tracked.msg
            image_rect.msg
            image_rect_ranged.msg
            lane.msg
            ndt_stat.msg
            obj_label.msg
            obj_pose.msg
            projection_matrix.msg
            vscan_tracked.msg
            vscan_tracked_array.msg
            waypoint.msg
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
            config/ConfigWaypointReplanner.msg
            config/ConfigWaypointFollower.msg
            accel_cmd.msg
            adjust_xy.msg
            brake_cmd.msg
            indicator_cmd.msg
            lamp_cmd.msg
            steer_cmd.msg
            traffic_light.msg
            state_cmd.msg
            state.msg
            ## Sync
            Sync_time_monitor.msg
            Sync_time_diff.msg

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
