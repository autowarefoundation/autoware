# Behavior Velocity Planner

## Overview

`behavior_velocity_planner` is a planner that adjust velocity based on the traffic rules.
It loads modules as plugins. Please refer to the links listed below for detail on each module.

![Architecture](./docs/BehaviorVelocityPlanner-Architecture.drawio.svg)

- [Blind Spot](../behavior_velocity_blind_spot_module/docs/blind-spot-design.md)
- [Crosswalk](../behavior_velocity_crosswalk_module/docs/crosswalk-design.md)
- [Detection Area](../behavior_velocity_detection_area_module/docs/detection-area-design.md)
- [Intersection](../behavior_velocity_intersection_module/docs/intersection-design.md)
- [MergeFromPrivate](../behavior_velocity_intersection_module/docs/merge-from-private-design.md)
- [Stop Line](../behavior_velocity_stop_line_module/docs/stop-line-design.md)
- [Virtual Traffic Light](../behavior_velocity_virtual_traffic_light_module/docs/virtual-traffic-light-design.md)
- [Traffic Light](../behavior_velocity_traffic_light_module/docs/traffic-light-design.md)
- [Occlusion Spot](../behavior_velocity_occlusion_spot_module/docs/occlusion-spot-design.md)
- [No Stopping Area](../behavior_velocity_no_stopping_area_module/docs/no-stopping-area-design.md)
- [Run Out](../behavior_velocity_run_out_module/docs/run-out-design.md)
- [Speed Bump](../behavior_velocity_speed_bump_module/docs/speed-bump-design.md)
- [Out of Lane](../behavior_velocity_out_of_lane_module/docs/out-of-lane-design.md)
- [No Drivable Lane](../behavior_velocity_no_drivable_lane_module/docs/no-drivable-lane-design.md)

When each module plans velocity, it considers based on `base_link`(center of rear-wheel axis) pose.
So for example, in order to stop at a stop line with the vehicles' front on the stop line, it calculates `base_link` position from the distance between `base_link` to front and modifies path velocity from the `base_link` position.

![set_stop_velocity](./docs/set_stop_velocity.drawio.svg)

## Input topics

| Name                                      | Type                                                   | Description                                                                                                                     |
| ----------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| `~input/path_with_lane_id`                | autoware_auto_planning_msgs::msg::PathWithLaneId       | path with lane_id                                                                                                               |
| `~input/vector_map`                       | autoware_auto_mapping_msgs::msg::HADMapBin             | vector map                                                                                                                      |
| `~input/vehicle_odometry`                 | nav_msgs::msg::Odometry                                | vehicle velocity                                                                                                                |
| `~input/dynamic_objects`                  | autoware_auto_perception_msgs::msg::PredictedObjects   | dynamic objects                                                                                                                 |
| `~input/no_ground_pointcloud`             | sensor_msgs::msg::PointCloud2                          | obstacle pointcloud                                                                                                             |
| `~/input/compare_map_filtered_pointcloud` | sensor_msgs::msg::PointCloud2                          | obstacle pointcloud filtered by compare map. Note that this is used only when the detection method of run out module is Points. |
| `~input/traffic_signals`                  | autoware_auto_perception_msgs::msg::TrafficSignalArray | traffic light states                                                                                                            |

## Output topics

| Name                   | Type                                      | Description                            |
| ---------------------- | ----------------------------------------- | -------------------------------------- |
| `~output/path`         | autoware_auto_planning_msgs::msg::Path    | path to be followed                    |
| `~output/stop_reasons` | tier4_planning_msgs::msg::StopReasonArray | reasons that cause the vehicle to stop |

## Node parameters

| Parameter              | Type                 | Description                                                                         |
| ---------------------- | -------------------- | ----------------------------------------------------------------------------------- |
| `launch_modules`       | vector&lt;string&gt; | module names to launch                                                              |
| `forward_path_length`  | double               | forward path length                                                                 |
| `backward_path_length` | double               | backward path length                                                                |
| `max_accel`            | double               | (to be a global parameter) max acceleration of the vehicle                          |
| `system_delay`         | double               | (to be a global parameter) delay time until output control command                  |
| `delay_response_time`  | double               | (to be a global parameter) delay time of the vehicle's response to control commands |
