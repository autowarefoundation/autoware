# Avoidance by lane change design

This is a sub-module to avoid obstacles by lane change maneuver.

## Purpose / Role

This module is designed as one of the obstacle avoidance features and generates a lane change path if the following conditions are satisfied.

- Exist lane changeable lanelet.
- Exist avoidance target objects on ego driving lane.

![avoidance_by_lane_change](./images/avoidance_by_lane_change.svg)

## Inner-workings / Algorithms

Basically, this module is implemented by reusing the avoidance target filtering logic of the existing [Static Object Avoidance Module](../autoware_behavior_path_static_obstacle_avoidance_module/README.md) and the path generation logic of the [Normal Lane Change Module](../autoware_behavior_path_lane_change_module/README.md). On the other hand, the conditions under which the module is activated differ from those of a normal avoidance module.

Check that the following conditions are satisfied after the filtering process for the avoidance target.

### Number of the avoidance target objects

This module is launched when the number of avoidance target objects on **EGO DRIVING LANE** is greater than `execute_object_num`. If there are no avoidance targets in the ego driving lane or their number is less than the parameter, the obstacle is avoided by normal avoidance behavior (if the normal avoidance module is registered).

![trigger_1](./images/avoidance_by_lc_trigger_1.svg)

### Lane change end point condition

Unlike the normal avoidance module, which specifies the shift line end point, this module does not specify its end point when generating a lane change path. On the other hand, setting `execute_only_when_lane_change_finish_before_object` to `true` will activate this module only if the lane change can be completed before the avoidance target object.

Although setting the parameter to `false` would increase the scene of avoidance by lane change, it is assumed that sufficient lateral margin may not be ensured in some cases because the vehicle passes by the side of obstacles during the lane change.

![trigger_2](./images/avoidance_by_lc_trigger_2.svg)

## Parameters

| Name                                               | Unit | Type   | Description                                                                                                                              | Default value |
| :------------------------------------------------- | ---- | ------ | ---------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| execute_object_num                                 | [-]  | int    | Number of avoidance target objects on ego driving lane is greater than this value, this module will be launched.                         | 1             |
| execute_object_longitudinal_margin                 | [m]  | double | [maybe unused] Only when distance between the ego and avoidance target object is longer than this value, this module will be launched.   | 0.0           |
| execute_only_when_lane_change_finish_before_object | [-]  | bool   | If this flag set `true`, this module will be launched only when the lane change end point is **NOT** behind the avoidance target object. | true          |
