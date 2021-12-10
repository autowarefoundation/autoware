# awapi_awiv_adapter

✓: confirmed, (blank): not confirmed

## get topic

### /awapi/vehicle/get/status

- get vehicle status
- MessageType: awapi_awiv_adapter/AwapiVehicleStatus

| ✓   | type                     | name                     | unit                                       | note                                     |
| --- | :----------------------- | :----------------------- | :----------------------------------------- | :--------------------------------------- |
| ✓   | std_msgs/Header          | header                   |                                            |                                          |
| ✓   | geometry_msgs/Pose       | pose                     | position:[m]                               |                                          |
| ✓   | awapi_awiv_adapter/Euler | eulerangle               | [rad]                                      | roll/pitch/yaw                           |
|     | geographic_msgs/GeoPoint | geo_point                |                                            | lat/lon/alt                              |
| ✓   | float64                  | velocity                 | [m/s]                                      |                                          |
| ✓   | float64                  | acceleration             | [m/ss]                                     | calculate from velocity in awapi_adapter |
| ✓   | float64                  | steering                 | [rad]                                      |                                          |
| ✓   | float64                  | steering_velocity        | [rad/s]                                    | calculate from steering in awapi_adapter |
| ✓   | float64                  | angular_velocity         | [rad/s]                                    |                                          |
|     | int32                    | gear                     | according to tier4_vehicle_msgs/Shift      |                                          |
|     | float32                  | energy_level             |                                            | available only for golf-cart             |
| ✓   | int32                    | turn_signal              | according to tier4_vehicle_msgs/TurnSignal |                                          |
| ✓   | float64                  | target_velocity          | [m/s]                                      |                                          |
| ✓   | float64                  | target_acceleration      | [m/ss]                                     |                                          |
| ✓   | float64                  | target_steering          | [rad]                                      |                                          |
| ✓   | float64                  | target_steering_velocity | [rad/s]                                    |                                          |

### /awapi/autoware/get/status

- get autoware status
- MessageType: awapi_awiv_adapter/AwapiVehicleStatus

| ✓   | type                                | name                 | unit                                        | note                                                                                                                                                                        |
| --- | :---------------------------------- | :------------------- | :------------------------------------------ | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ✓   | std_msgs/Header                     | header               |                                             |                                                                                                                                                                             |
| ✓   | string                              | autoware_state       |                                             |                                                                                                                                                                             |
| ✓   | int32                               | control_mode         | according to tier4_vehicle_msgs/ControlMode | manual/auto (changed by /awapi/autoware/put/engage)                                                                                                                         |
|     | int32                               | gate_mode            | tier4_vehicle_msgs/GateMode                 | auto/remote (it is valid only when control_mode=auto))                                                                                                                      |
| ✓   | bool                                | emergency_stopped    | True in emergency mode                      |                                                                                                                                                                             |
| ✓   | float32                             | current_max_velocity | [m/s]                                       |                                                                                                                                                                             |
|     | tier4_system_msgs/HazardStatus      | hazard_status        |                                             | system hazard status                                                                                                                                                        |
| ✓   | tier4_planning_msgs/StopReasonArray | stop_reason          |                                             | "stop_pose" represents the position of "base_link" (not the head of the car)                                                                                                |
| ✓   | diagnostic_msgs/DiagnosticStatus[]  | diagnostics          |                                             | output only diag. of leaf node (diag. of parent node are cut)                                                                                                               |
| ✓   | diagnostic_msgs/DiagnosticStatus[]  | error_diagnostics    |                                             | diagnostics that are the cause of system emergency                                                                                                                          |
| ✓   | bool                                | arrived_goal         |                                             | True if the autoware_state is changed from Driving to ArrivedGoal or WaitingForRoute. False if the autoware_state is changed to WaitingForEngage or Driving. Default False. |

- specification of stop_reason
  - stop_reason is output only when the following conditions are met.
    - stop_point in stop_reason is close to /planning/scenario_planning/trajectory (within 10m).
    - The distance between current position and stop_point is less than stop_reason_thresh_dist.

### /awapi/autoware/get/route

- get route
- MessageType: tier4_planning_msgs/Route

| ✓   | type                      | name | unit | note |
| --- | :------------------------ | :--- | :--- | :--- |
| ✓   | tier4_planning_msgs/Route |      |      |      |

### /awapi/autoware/get/stop_speed_exceeded

- get flag of exceeding stop speed or not
  - True: exceed the stop speed ( = "cannot stop before the stop line")
  - False: not exceed the stop speed ( = "no stop line in the trajectory" or "possible to stop before the stop line" )
- MessageType: tier4_planning_msgs/StopSpeedExceedStatus

| ✓   | type                                      | name | unit | note |
| --- | :---------------------------------------- | :--- | :--- | :--- |
|     | tier4_planning_msgs/StopSpeedExceedStatus |      | -    |      |

### /awapi/prediction/get/objects

- get predicted object
- MessageType: tier4_api_msgs/DynamicObjectArray

| ✓   | type                              | name | unit | note |
| --- | :-------------------------------- | :--- | :--- | :--- |
| ✓   | tier4_api_msgs/DynamicObjectArray |      |      |      |

### /awapi/lane_change/get/status

- get lane change information
- MessageType: awapi_awiv_adapter/LaneChangeStatus

| ✓   | type                     | name                        | unit                                  | note                                                                               |
| --- | :----------------------- | :-------------------------- | :------------------------------------ | :--------------------------------------------------------------------------------- |
|     | std_msgs/Header          | header                      |                                       |                                                                                    |
|     | bool                     | force_lane_change_available | True when lane change is available    | available: Physically lane changeable state (do not consider other vehicle)        |
|     | bool                     | lane_change_ready           | True when lane change is ready        | ready: State that ego-vehicle can change lane without collision with other vehicle |
|     | tier4_planning_msgs/Path | candidate_path              | according to tier4_planning_msgs/Path |                                                                                    |

### /awapi/object_avoidance/get/status

- get obstacle avoidance information
- MessageType: awapi_awiv_adapter/ObstacleAvoidanceStatus

| ✓   | type                           | name                     | unit                                        | note                                                  |
| --- | :----------------------------- | :----------------------- | :------------------------------------------ | :---------------------------------------------------- |
|     | std_msgs/Header                | header                   |                                             |                                                       |
|     | bool                           | obstacle_avoidance_ready | True when obstacle avoidance is ready       |                                                       |
|     | tier4_planning_msgs/Trajectory | candidate_path           | according to tier4_planning_msgs/Trajectory | Msg type is different from lane change candidate path |

### /awapi/traffic_light/get/traffic_signals

- get recognition result of traffic light
- MessageType: autoware_auto_perception_msgs/msg/TrafficSignalArray

| ✓   | type                                                 | name | unit | note |
| --- | :--------------------------------------------------- | :--- | :--- | :--- |
|     | autoware_auto_perception_msgs/msg/TrafficSignalArray |      |      |      |

### /awapi/traffic_light/get/nearest_traffic_signal

- get recognition result of nearest traffic light
- MessageType: autoware_auto_perception_msgs/LookingTrafficSignal

|     | type                                                 | name       | unit | note                                                          |
| --- | :--------------------------------------------------- | :--------- | :--- | :------------------------------------------------------------ |
|     | std_msgs/Header                                      | header     |      |                                                               |
|     | autoware_auto_perception_msgs/TrafficSignalWithJudge | perception |      | traffic light information from autoware perception module     |
|     | autoware_auto_perception_msgs/TrafficSignalWithJudge | external   |      | traffic light information from external tool/module           |
|     | autoware_auto_perception_msgs/TrafficSignalWithJudge | final      |      | traffic light information used by the planning module finally |

- The contents of TrafficSignalWithJudge.msg is following.

|     | type                                        | name   | unit                 | note                                                           |
| --- | :------------------------------------------ | :----- | :------------------- | :------------------------------------------------------------- |
|     | autoware_auto_perception_msgs/TrafficSignal | signal |                      | traffic light color/arrow                                      |
|     | uint8                                       | judge  | 0:NONE, 1:STOP, 2:GO | go/stop judgment based on the color/arrow of the traffic light |

### /awapi/vehicle/get/door

- get door status
- MessageType: tier4_api_msgs/DoorStatus.msg

| ✓   | type                      | name   | unit                                                                                     | note                                        |
| --- | :------------------------ | :----- | :--------------------------------------------------------------------------------------- | :------------------------------------------ |
|     | tier4_api_msgs/DoorStatus | status | 0:UNKNOWN, 1:DOOR_OPENED, 2:DOOR_CLOSED 3:DOOR_OPENING, 4:DOOR_CLOSING, 5:NOT_APPLICABLE | available only for the vehicle using pacmod |

- Now, available status is following: (0:UNKNOWN, 1:DOOR_OPENED, 2:DOOR_CLOSED, 5:NOT_APPLICABLE ).
- 5 (NOT_APPLICABLE) is published if the pacmod is not used
- Due to the specifications of pacmod, the last door open / close command is published as the status.
- The status is 0 (UNKNOWN) until the door open / close command is published once.

## put topic

### /awapi/vehicle/put/velocity

- set upper velocity
- MessageType: tier4_api_msgs/VelocityLimit

| ✓   | type                         | name | unit | note         |
| --- | :--------------------------- | :--- | :--- | :----------- |
| ✓   | tier4_api_msgs/VelocityLimit |      |      | max velocity |

### /awapi/vehicle/put/stop

- set temporary stop signal
- MessageType: tier4_api_msgs/StopCommand
- Specification

  - send True: send upper velocity to 0
  - send False: resend last received upper velocity
    - (if upper velocity have never received, send _default_max_velocity_ value.)
    - _default_max_velocity_ refers to the param: _/planning/scenario_planning/motion_velocity_optimizer/max_velocity_

  | ✓   | type                       | name | unit | note |
  | --- | :------------------------- | :--- | :--- | :--- |
  | ✓   | tier4_api_msgs/StopCommand |      |      |      |

### /awapi/autoware/put/gate_mode

- send gate mode (auto/remote)
- MessageType: tier4_control_msgs/GateMode

| ✓   | type                        | name | unit | note |
| --- | :-------------------------- | :--- | :--- | :--- |
|     | tier4_control_msgs/GateMode |      |      |      |

### /awapi/autoware/put/engage

- send engage signal (both of autoware/engage and vehicle/engage)
- MessageType: tier4_vehicle_msgs/Engage

| ✓   | type                      | name | unit | note |
| --- | :------------------------ | :--- | :--- | :--- |
| ✓   | tier4_vehicle_msgs/Engage |      |      |      |

### /awapi/autoware/put/goal

- send goal pose
- MessageType: geometry_msgs/PoseStamped

| ✓   | type                      | name | unit | note |
| --- | :------------------------ | :--- | :--- | :--- |
|     | geometry_msgs/PoseStamped |      |      |      |

### /awapi/autoware/put/route

- send route
- MessageType: tier4_planning_msgs/Route

| ✓   | type                      | name | unit | note |
| --- | :------------------------ | :--- | :--- | :--- |
| ✓   | tier4_planning_msgs/Route |      |      |      |

### /awapi/lane_change/put/approval

- send lane change approval flag
- send True: start lane change when **lane_change_ready** is true
- MessageType: tier4_planning_msgs/LaneChangeCommand

| ✓   | type                                      | name | unit | note |
| --- | :---------------------------------------- | :--- | :--- | :--- |
|     | tier4_planning_msgs/msg/LaneChangeCommand |      |      |      |

### /awapi/lane_change/put/force

- send force lane change flag
- send True: start lane change when **force_lane_change_available** is true
- MessageType: tier4_planning_msgs/LaneChangeCommand

| ✓   | type                                  | name | unit | note |
| --- | :------------------------------------ | :--- | :--- | :--- |
|     | tier4_planning_msgs/LaneChangeCommand |      |      |      |

### /awapi/object_avoidance/put/approval

- send object avoidance approval flag
- MessageType: tier4_planning_msgs/EnableAvoidance

| ✓   | type                                | name | unit | note |
| --- | :---------------------------------- | :--- | :--- | :--- |
|     | tier4_planning_msgs/EnableAvoidance |      |      |      |

### /awapi/object_avoidance/put/force

- send force object avoidance flag
- <font color="Red">**not implemented (Autoware does not have corresponded topic)**</font>

| ✓   | type | name | unit | note |
| --- | :--- | :--- | :--- | :--- |

### /awapi/traffic_light/put/traffic_signals

- Overwrite the recognition result of traffic light
- MessageType: autoware_auto_perception_msgs/TrafficSignalArray

| ✓   | type                                             | name | unit | note |
| --- | :----------------------------------------------- | :--- | :--- | :--- |
|     | autoware_auto_perception_msgs/TrafficSignalArray |      |      |      |

### /awapi/vehicle/put/door

- send door command
- MessageType: tier4_api_msgs/DoorCommand
  - send True: open door
  - send False: close door

| ✓   | type                       | name | unit | note                                        |
| --- | :------------------------- | :--- | :--- | :------------------------------------------ |
|     | tier4_api_msgs/DoorCommand |      |      | available only for the vehicle using pacmod |

### /awapi/autoware/put/crosswalk_states

- send crosswalk status
  - forcibly rewrite the internal state of crosswalk module
- MessageType: tier4_api_msgs/CrossWalkStatus

| ✓   | type            | name   | unit                     | note |
| --- | :-------------- | :----- | :----------------------- | :--- |
|     | std_msgs/Header | header |                          |      |
|     | int32           | status | 0:STOP, 1:GO, 2:SLOWDOWN |      |

### /awapi/autoware/put/intersection_states

- send intersection status
  - forcibly rewrite the internal state of intersection module
- MessageType: tier4_api_msgs/CrosswalkStatus

| ✓   | type            | name   | unit         | note |
| --- | :-------------- | :----- | :----------- | :--- |
|     | std_msgs/Header | header |              |      |
|     | int32           | status | 0:STOP, 1:GO |      |

### /awapi/autoware/put/expand_stop_range

- send expand range of the polygon used by obstacle stop [m]
- MessageType: tier4_planning_msgs/ExpandStopRange

| ✓   | type                                | name | unit | note |
| --- | :---------------------------------- | :--- | :--- | :--- |
|     | tier4_planning_msgs/ExpandStopRange |      |      |      |

---
