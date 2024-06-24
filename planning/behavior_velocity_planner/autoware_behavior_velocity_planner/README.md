# Behavior Velocity Planner

## Overview

`behavior_velocity_planner` is a planner that adjust velocity based on the traffic rules.
It loads modules as plugins. Please refer to the links listed below for detail on each module.

![Architecture](./docs/BehaviorVelocityPlanner-Architecture.drawio.svg)

- [Blind Spot](../autoware_behavior_velocity_blind_spot_module/README.md)
- [Crosswalk](../autoware_behavior_velocity_crosswalk_module/README.md)
- [Walkway](../autoware_behavior_velocity_walkway_module/README.md)
- [Detection Area](../autoware_behavior_velocity_detection_area_module/README.md)
- [Intersection](../autoware_behavior_velocity_intersection_module/README.md)
- [MergeFromPrivate](../autoware_behavior_velocity_intersection_module/README.md#merge-from-private)
- [Stop Line](../autoware_behavior_velocity_stop_line_module/README.md)
- [Virtual Traffic Light](../autoware_behavior_velocity_virtual_traffic_light_module/README.md)
- [Traffic Light](../autoware_behavior_velocity_traffic_light_module/README.md)
- [Occlusion Spot](../autoware_behavior_velocity_occlusion_spot_module/README.md)
- [No Stopping Area](../autoware_behavior_velocity_no_stopping_area_module/README.md)
- [Run Out](../autoware_behavior_velocity_run_out_module/README.md)
- [Speed Bump](../autoware_behavior_velocity_speed_bump_module/README.md)

When each module plans velocity, it considers based on `base_link`(center of rear-wheel axis) pose.
So for example, in order to stop at a stop line with the vehicles' front on the stop line, it calculates `base_link` position from the distance between `base_link` to front and modifies path velocity from the `base_link` position.

![set_stop_velocity](./docs/set_stop_velocity.drawio.svg)

## Input topics

| Name                                      | Type                                                  | Description                                                                                                                     |
| ----------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `~input/path_with_lane_id`                | tier4_planning_msgs::msg::PathWithLaneId              | path with lane_id                                                                                                               |
| `~input/vector_map`                       | autoware_map_msgs::msg::LaneletMapBin                 | vector map                                                                                                                      |
| `~input/vehicle_odometry`                 | nav_msgs::msg::Odometry                               | vehicle velocity                                                                                                                |
| `~input/dynamic_objects`                  | autoware_perception_msgs::msg::PredictedObjects       | dynamic objects                                                                                                                 |
| `~input/no_ground_pointcloud`             | sensor_msgs::msg::PointCloud2                         | obstacle pointcloud                                                                                                             |
| `~/input/compare_map_filtered_pointcloud` | sensor_msgs::msg::PointCloud2                         | obstacle pointcloud filtered by compare map. Note that this is used only when the detection method of run out module is Points. |
| `~input/traffic_signals`                  | autoware_perception_msgs::msg::TrafficLightGroupArray | traffic light states                                                                                                            |

## Output topics

| Name                   | Type                                      | Description                            |
| ---------------------- | ----------------------------------------- | -------------------------------------- |
| `~output/path`         | autoware_planning_msgs::msg::Path         | path to be followed                    |
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

## Traffic Light Handling in sim/real

The handling of traffic light information varies depending on the usage. In the below table, the traffic signal topic element for the corresponding lane is denoted as `info`, and if `info` is not available, it is denoted as `null`.

| module \\ case                                                                                             | `info` is `null`         | `info` is not `null`                                                                                                                                                                                                                                                                 |
| :--------------------------------------------------------------------------------------------------------- | ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| intersection_occlusion(`is_simulation = *`) <ul> <li>`info` is the latest non-`null` information</li></ul> | GO(occlusion is ignored) | intersection_occlusion uses the latest non UNKNOWN observation in the queue up to present.<ul><li>If `info` is `GREEN or UNKNOWN`, occlusion is cared</li><li>If `info` is `RED or YELLOW`, occlusion is ignored(GO) </li> <li> NOTE: Currently timeout is not considered</li> </ul> |
| traffic_light(sim, `is_simulation = true`) <ul> <li>`info` is current information</li></ul>                | GO                       | traffic_light uses the perceived traffic light information at present directly. <ul><li>If `info` is timeout, STOP whatever the color is</li> <li>If `info` is not timeout, then act according to the color. If `info` is `UNKNOWN`, STOP</li></ul> {: rowspan=2}                    |
| traffic_light(real, `is_simulation = false`) <ul> <li>`info` is current information</li></ul>              | STOP                     | &#8288 {: style="padding:0"}                                                                                                                                                                                                                                                         |
| crosswalk with Traffic Light(`is_simulation = *`) <ul> <li>`info` is current information</li></ul>         | default                  | <ul> <li>If `disable_yield_for_new_stopped_object` is true, each sub scene_module ignore newly detected pedestrians after module instantiation.</li> <li>If `ignore_with_traffic_light` is true, occlusion detection is skipped.</li></ul>                                           |
| map_based_prediction(`is_simulation = *`) <ul> <li>`info` is current information</li></ul>                 | default                  | If a pedestrian traffic light is<ul> <li>RED, surrounding pedestrians are not predicted.</li> <li>GREEN, stopped pedestrians are not predicted.</li></ul>                                                                                                                            |
