# Dynamic avoidance design

## Purpose / Role

This is a module designed for avoiding obstacles which are running.
Static obstacles such as parked vehicles are dealt with by the avoidance module.

This module is under development.
In the current implementation, the dynamic obstacles to avoid is extracted from the drivable area.
Then the motion planner, in detail obstacle_avoidance_planner, will generate an avoiding trajectory.

## Overview of drivable area modification

### Filtering obstacles to avoid

The dynamics obstacles meeting the following condition will be avoided.

- The type is designated as `target_object.*`.
- The norm of the obstacle's velocity projected to the ego's path is smaller than `target_object.min_obstacle_vel`.
- The obstacle is in the next lane to the ego's lane, which will not cut-into the ego's lane according to the highest-prioritized predicted path.

### Drivable area modification

To realize dynamic obstacles for avoidance, the time dimension should be take into an account considering the dynamics.
However, it will make the planning problem much harder to solve.
Therefore, we project the time dimension to the 2D pose dimension.

Currently, the predicted paths of predicted objects are not so stable.
Therefore, instead of using the predicted paths, we assume that the obstacle will run parallel to the ego's path.

First, a maximum lateral offset to avoid is calculated as follows.
The polygon's width to extract from the drivable area is the obstacle width and double `drivable_area_generation.lat_offset_from_obstacle`.
We can limit the lateral shift offset by `drivable_area_generation.max_lat_offset_to_avoid`.

![drivable_area_extraction_width](../image/dynamic_avoidance/drivable_area_extraction_width.drawio.svg)

Then, extracting the same directional and opposite directional obstacles from the drivable area will work as follows considering TTC (time to collision).
Regarding the same directional obstacles, obstacles whose TTC is negative will be ignored (e.g. The obstacle is in front of the ego, and the obstacle's velocity is larger than the ego's velocity.).

Same directional obstacles
![same_directional_object](../image/dynamic_avoidance/same_directional_object.svg)

Opposite directional obstacles
![opposite_directional_object](../image/dynamic_avoidance/opposite_directional_object.svg)

## Parameters

| Name                                                                  | Unit  | Type   | Description                                                | Default value |
| :-------------------------------------------------------------------- | :---- | :----- | :--------------------------------------------------------- | :------------ |
| target_object.car                                                     | [-]   | bool   | The flag whether to avoid cars or not                      | true          |
| target_object.truck                                                   | [-]   | bool   | The flag whether to avoid trucks or not                    | true          |
| ...                                                                   | [-]   | bool   | ...                                                        | ...           |
| target_object.min_obstacle_vel                                        | [m/s] | double | Minimum obstacle velocity to avoid                         | 1.0           |
| drivable_area_generation.lat_offset_from_obstacle                     | [m]   | double | Lateral offset to avoid from obstacles                     | 0.8           |
| drivable_area_generation.max_lat_offset_to_avoid                      | [m]   | double | Maximum lateral offset to avoid                            | 0.5           |
| drivable_area_generation.overtaking_object.max_time_to_collision      | [s]   | double | Maximum value when calculating time to collision           | 3.0           |
| drivable_area_generation.overtaking_object.start_duration_to_avoid    | [s]   | double | Duration to consider avoidance before passing by obstacles | 4.0           |
| drivable_area_generation.overtaking_object.end_duration_to_avoid      | [s]   | double | Duration to consider avoidance after passing by obstacles  | 5.0           |
| drivable_area_generation.overtaking_object.duration_to_hold_avoidance | [s]   | double | Duration to hold avoidance after passing by obstacles      | 3.0           |
| drivable_area_generation.oncoming_object.max_time_to_collision        | [s]   | double | Maximum value when calculating time to collision           | 3.0           |
| drivable_area_generation.oncoming_object.start_duration_to_avoid      | [s]   | double | Duration to consider avoidance before passing by obstacles | 9.0           |
| drivable_area_generation.oncoming_object.end_duration_to_avoid        | [s]   | double | Duration to consider avoidance after passing by obstacles  | 0.0           |
