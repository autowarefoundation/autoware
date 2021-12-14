# ad_service_state_monitor

## Purpose

This node manages AutowareState transitions.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                               | Type                                                 | Description                                       |
| ---------------------------------- | ---------------------------------------------------- | ------------------------------------------------- |
| `/planning/mission_planning/route` | `autoware_auto_planning_msgs::msg::HADMapRoute`      | Subscribe route                                   |
| `/localization/kinematic_state`    | `nav_msgs::msg::Odometry`                            | Used to decide whether vehicle is stopped or not  |
| `/vehicle/state_report`            | `autoware_auto_vehicle_msgs::msg::ControlModeReport` | Used to check vehicle mode: autonomous or manual. |

### Output

| Name               | Type                                            | Description                                        |
| ------------------ | ----------------------------------------------- | -------------------------------------------------- |
| `/autoware/engage` | `autoware_auto_vehicle_msgs::msg::Engage`       | publish disengage flag on AutowareState transition |
| `/autoware/state`  | `autoware_auto_system_msgs::msg::AutowareState` | publish AutowareState                              |

## Parameters

### Node Parameters

| Name          | Type | Default Value | Explanation            |
| ------------- | ---- | ------------- | ---------------------- |
| `update_rate` | int  | `10`          | Timer callback period. |

### Core Parameters

| Name                      | Type   | Default Value | Explanation                                                                |
| ------------------------- | ------ | ------------- | -------------------------------------------------------------------------- |
| `th_arrived_distance_m`   | double | 1.0           | threshold distance to check if vehicle has arrived at the route's endpoint |
| `th_stopped_time_sec`     | double | 1.0           | threshold time to check if vehicle is stopped                              |
| `th_stopped_velocity_mps` | double | 0.01          | threshold velocity to check if vehicle is stopped                          |
| `disengage_on_route`      | bool   | true          | send disengage flag or not when the route is subscribed                    |
| `disengage_on_goal`       | bool   | true          | send disengage flag or not when the vehicle is arrived goal                |

## Assumptions / Known limits

TBD.
