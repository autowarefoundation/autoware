# Dynamic avoidance design

## Purpose / Role

This is a module designed for avoiding obstacles which are running.
Static obstacles such as parked vehicles are dealt with by the avoidance module.

This module is under development.
In the current implementation, the dynamic obstacles to avoid is extracted from the drivable area.
Then the motion planner, in detail obstacle_avoidance_planner, will generate an avoiding trajectory.

### Parameters

| Name                                                               | Unit  | Type   | Description                                                 | Default value |
| :----------------------------------------------------------------- | :---- | :----- | :---------------------------------------------------------- | :------------ |
| target_object.car                                                  | [-]   | bool   | The flag whether to avoid cars or not                       | true          |
| target_object.truck                                                | [-]   | bool   | The flag whether to avoid trucks or not                     | true          |
| ...                                                                | [-]   | bool   | ...                                                         | ...           |
| target_object.min_obstacle_vel                                     | [m/s] | double | Minimum obstacle velocity to avoid                          | 1.0           |
| drivable_area_generation.lat_offset_from_obstacle                  | [m]   | double | Lateral offset to avoid from obstacles                      | 0.8           |
| drivable_area_generation.time_to_avoid_same_directional_object     | [s]   | double | Elapsed time for avoiding the same directional obstacle     | 5.0           |
| drivable_area_generation.time_to_avoid_opposite_directional_object | [s]   | double | Elapsed time for avoiding the opposite directional obstacle | 6.0           |
| drivable_area_generation.max_lat_offset_to_avoid                   | [m]   | double | Maximum lateral offset to avoid                             | 0.5           |
