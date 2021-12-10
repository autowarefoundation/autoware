# goal_distance_calculator

## Purpose

This node publishes deviation of self-pose from goal pose.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                               | Type                                      | Description           |
| ---------------------------------- | ----------------------------------------- | --------------------- |
| `/planning/mission_planning/route` | `autoware_auto_planning_msgs::msg::Route` | Used to get goal pose |
| `/tf`                              | `tf2_msgs/TFMessage`                      | TF (self-pose)        |

### Output

| Name                     | Type                                    | Description                                                   |
| ------------------------ | --------------------------------------- | ------------------------------------------------------------- |
| `deviation/lateral`      | `tier4_debug_msgs::msg::Float64Stamped` | publish lateral deviation of self-pose from goal pose[m]      |
| `deviation/longitudinal` | `tier4_debug_msgs::msg::Float64Stamped` | publish longitudinal deviation of self-pose from goal pose[m] |
| `deviation/yaw`          | `tier4_debug_msgs::msg::Float64Stamped` | publish yaw deviation of self-pose from goal pose[rad]        |
| `deviation/yaw_deg`      | `tier4_debug_msgs::msg::Float64Stamped` | publish yaw deviation of self-pose from goal pose[deg]        |

## Parameters

### Node Parameters

| Name          | Type   | Default Value | Explanation                 |
| ------------- | ------ | ------------- | --------------------------- |
| `update_rate` | double | 10.0          | Timer callback period. [Hz] |

### Core Parameters

| Name      | Type | Default Value | Explanation                                |
| --------- | ---- | ------------- | ------------------------------------------ |
| `oneshot` | bool | true          | publish deviations just once or repeatedly |

## Assumptions / Known limits

TBD.
