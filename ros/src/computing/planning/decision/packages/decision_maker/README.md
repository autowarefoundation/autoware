# Decision Maker

### Overview
Autoware package that visualize internal state and publish some commands.

### Parameters from runtime manager
Parameter|Type|Description
--|---|--
enable_display_marker|Bool|(default: *false*)<br>not used
auto_mission_reload|Bool|(default: *false*)<br>If this is set true, decision maker automatically reloads mission after mission is completed.
use_management_system|Bool|(default: *false*)<br>This must be true in order to incoorporate with [Autoware Management System](https://github.com/CPFL/AMS)
disuse_vector_map|Bool|(default: *false*)<br> If set *true*, decision_maker will exit "MapInitState" even if vector map is not successfully loaded.
num_of_steer_behind|Int|(default: *20*)<br> lookup distance along waypoints to determine steering state(straight, turning right, or turning left)
change_threshold_dist|Double|(default: *1*)<br> This is relevent only if *use_management_system* is *true*.<br> If the distance from vehicle to closest waypoint in the new mission is further than *change_threshold_dist* [m], mission change fails.
change_threshold_angle|Double|(default:*15*)<br>This is relevent only if *use_management_system* is *true*.<br> If the angle from vehicle to closest waypoint in the new mission is further than this *change_threshold_dist* [deg], mission change fails.
time_to_avoidance|Double|(default: *3*)<br> If the vehicle is stuck for *time_to_avoidance* seconds (e.g. due to obstacles), the state transits to from "Go" to "Avoidance".
goal_threshold_dist|Double|(default: *3*)<br> Threshold used to check if vehicle has reached to the goal (i.e. end of waypoints). The vehicle must be less than *goal_threshold_dist* [m] to the goal.
goal_threshold_vel|Double|(default: *0.1*)<br> Threshold used to check if vehicle has reached to the goal (i.e. end of waypoints). The vehicle must be less than *goal_threshold_vel* [m/s] to be treated as goal arrival.


### Subscribed topics
Topic|Type|Objective
--|---|--
/based/lane_waypoints_array|autoware_msgs/LaneArray|waypoints for the vehicle to follow. (e.g. waypoints given from waypoint_loader node)
/change_flag|std_msgs/Int32|Vehicle will try to change lane if this flag is set. Publishes /lamp_cmd to change blinkers according to this flag. (0=straight, 1=right, 2=left)
/closest_waypoint|std_msgs/Int32|Closest waypoint index in waypoints given by /based/lane_waypoints_array.
/config/decision_maker|autoware_config_msgs::ConfigDecisionMaker|Parameters set from runtime manager
/current_pose|geometry_msgs/PoseStamped|Current pose of vehicle
/current_velocity|geometry_msgs/TwistStamped|Current velocity of vehicle
/filtered_points|sensor_msgs/PointCloud2|Used to check if sensor data is ready. This is meant to give pointcloud data used for ndt_matching.
/lane_waypoints_array|autoware_msgs/LaneArray|
/final_waypoints|autoware_msgs/Lane|
/obstacle_waypoint|std_msgs/Int32|Obstacle waypoint index. Used in "Go" state.
/state_cmd|std_msgs/String|Decision_maker will try to transit state according to key given by through this topic.
/vector_map_info/area|vector_map_msgs/AreaArray|Area information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/cross_road|vector_map_msgs/CrossRoadArray|Cross road information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/line|vector_map_msgs/LineArray|Line information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/point|vector_map_msgs/PointArray|Point information from vector map.<br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.  
/vector_map_info/road_sign|vector_map_msgs/RoadSignArray|Road sign information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/stop_line|vector_map_msgs/StopLineArray|Stop line information from vector map.<br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/vector|vector_map_msgs/VectorArray|Vector information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.


### Published topics
Topic|Type|Objective
--|---|--
/decision_maker/available_transition|std_msgs/String|
/decision_maker/operator_help_text|jsk_rviz_plugins/OverlayText|
/decision_maker/state|std_msgs/String|
/lamp_cmd|autoware_msgs/LampCmd|
/lane_waypoints_array|autoware_msgs/LaneArray|
/light_color_managed|autoware_msgs/TrafficLight|
/state/cross_inside_marker|visualization_msgs/Marker|
/state/cross_road_marker|visualization_msgs/MarkerArray|
/state/crossroad_bbox|jsk_recognition_msgs/BoundingBoxArray|
/state/overlay_text|jsk_rviz_plugins/OverlayText|
/state/stopline_target|visualization_msgs/Marker|
/state/stopline_wpidx|std_msgs/Int32|
/target_velocity_array|std_msgs/Float64MultiArray|

### Requirements of each state or behavior

- ##### Initialize

State name|Required topic|Description
--|---|--
Init|-|The parent state of the following states.
SensorInit|/filtered_points|
MapInit||
LocalizationInit|/current_pose|
PlanningInit|/closest_waypoint|
VehicleInit|-|


- ##### Ready to driving
  1. Launch Autoware and localization on real vehicle or wf_simulator
  2. Launch `decision_maker` and lane_planner, astar_planner, waypoint_follower
  3. When `VehicleReady` and `WaitOrder` state, launch `waypoint_loader`
  4. On `DriveReady` state, push `Engage` button on DecisionMakerPannel
  5. When the vehicle reaches the end of waypoint and stops, state Mission state transits to `WaitOrder` via the `MissionComplete`
  6. You can repeat from 3. with other waypoint

- ##### Move backward

- ##### Lane change
  1. Start Normal driving with waypoint files necessary for lane change
  2. On `CheckLeft` or `RightLane` state, push `Execute LaneChange` button on DecisionMakerPannel
  3. The vehicle start lane change

- ##### Stop at stop line

- ##### Driving mission change
  1. Prepare waypoint files to change, and start Normal driving
  2. On `Driving` state, push `Request mission change` button on DecisionMakerPannel
  3. When the state becomes `DrivingMissionChange`, please load another waypoint for change
  4. If it is possible to change the waypoint will switch, otherwise it will not be changed

- ##### Obstacle avoidance

- ##### Pull over & Pull out

- ##### Emergency stop
