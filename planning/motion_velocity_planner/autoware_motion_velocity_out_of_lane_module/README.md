# Out of Lane

## Role

There are cases where the ego vehicle footprint goes out of the driving lane,
for example when taking a narrow turn with a large vehicle.
The `out_of_lane` module adds deceleration and stop points to the ego trajectory in order to prevent collisions from occurring in these out of lane cases.

## Activation

This module is activated if the launch parameter `launch_out_of_lane_module` is set to true.

## Inner-workings / Algorithms

This module calculates if out of lane collisions occur and insert stop point before the collisions if necessary.

The algorithm assumes the input ego trajectory contains accurate `time_from_start`
values in order to calculate accurate time to collisions with the predicted objects.

Next we explain the inner-workings of the module in more details.

### 1. Ego trajectory footprints

In this first step, the ego footprint is projected at each trajectory point and its size is modified based on the `ego.extra_..._offset` parameters.

The length of the trajectory used for generating the footprints is limited by the `max_arc_length` parameter.

![ego_footprints](./docs/footprints.png)

### 2. Ego lanelets

In the second step, we calculate the lanelets followed by the ego trajectory.
We select all lanelets crossed by the trajectory linestring (sequence of trajectory points), as well as their preceding lanelets.

![ego_lane](./docs/ego_lane.png)

In the debug visualization the combination of all ego lanelets is shown as a blue polygon.

### 3. Out of lane areas

Next, for each trajectory point, we create the corresponding out of lane areas by subtracting the ego lanelets (from step 2) from the trajectory point footprint (from step 1).
Each area is associated with the lanelets overlapped by the area and with the corresponding ego trajectory point.

![out_of_lane_areas](./docs/out_of_lane_areas.png)

In the debug visualization, the out of lane area polygon is connected to the corresponding trajectory point by a line.

### 4. Predicted objects filtering

We filter objects and their predicted paths with the following conditions:

- ignore objects with a speed bellow the `minimum_velocity` parameter;
- ignore objects coming from behind the ego vehicle if parameter `ignore_behind_ego` is set to true;
- ignore predicted paths whose confidence value is bellow the `predicted_path_min_confidence` parameter;
- cut the points of predicted paths going beyond the stop line of a red traffic light if parameter `cut_predicted_paths_beyond_red_lights` is set to `true`.

| `cut_predicted_paths_beyond_red_lights = false` | `cut_predicted_paths_beyond_red_lights = true` |
| :---------------------------------------------: | :--------------------------------------------: |
|        ![](./docs/path_green_light.png)         |         ![](./docs/path_red_light.png)         |

In the debug visualization, the filtered predicted paths are shown in green and the stop lines of red traffic lights are shown in red.

### 5. Time to collisions

For each out of lane area, we calculate the times when a dynamic object will overlap the area based on its filtered predicted paths.

In the case where parameter `mode` is set to `threshold` and the calculated time is less than `threshold.time_threshold` parameter, then we decide to avoid the out of lane area.

In the case where parameter `mode` is set to `ttc`,
we calculate the time to collision by comparing the predicted time of the object with the `time_from_start` field contained in the trajectory point.
If the time to collision is bellow the `ttc.threshold` parameter value, we decide to avoid the out of lane area.

![ttcs](./docs/ttcs.png)

In the debug visualization, the ttc (in seconds) is displayed on top of its corresponding trajectory point.
The color of the text is red if the collision should be avoided and green otherwise.

### 6. Calculate the stop or slowdown point

First, the minimum stopping distance of the ego vehicle is calculated based on the jerk and deceleration constraints set by the velocity smoother parameters.

We then search for the furthest pose along the trajectory where the ego footprint stays inside of the ego lane (calculate in step 2) and constraint the search to be between the minimum stopping distance and the 1st trajectory point with a collision to avoid (as determined in the previous step).
The search is done by moving backward along the trajectory with a distance step set by the `action.precision` parameter.

We first do this search for a footprint expanded with the `ego.extra_..._offset`, `action.longitudinal_distance_buffer` and `action.lateral_distance_buffer` parameters.
If no valid pose is found, we search again while only considering the extra offsets but without considering the distance buffers.
If still no valid pose is found, we use the base ego footprint without any offset.
In case no pose is found, we fallback to using the pose before the detected collision without caring if it is out of lane or not.

Whether it is decided to slow down or stop is determined by the distance between the ego vehicle and the trajectory point to avoid.
If this distance is bellow the `actions.slowdown.threshold`, a velocity of `actions.slowdown.velocity` will be used.
If the distance is bellow the `actions.stop.threshold`, a velocity of `0`m/s will be used.

![avoid_collision](./docs/ttcs_avoid.png)

### About stability of the stop/slowdown pose

As the input trajectory can change significantly between iterations,
it is expected that the decisions of this module will also change.
To make the decision more stable, a stop or slowdown pose is used for a minimum duration set by the `action.min_duration` parameter.
If during that time a new pose closer to the ego vehicle is generated, then it replaces the previous one.
Otherwise, the stop or slowdown pose will only be discarded after no out of lane collision is detection for the set duration.

## Module Parameters

| Parameter                     | Type   | Description                                                                       |
| ----------------------------- | ------ | --------------------------------------------------------------------------------- |
| `mode`                        | string | [-] mode used to consider a dynamic object. Candidates: threshold, intervals, ttc |
| `skip_if_already_overlapping` | bool   | [-] if true, do not run this module when ego already overlaps another lane        |
| `max_arc_length`              | double | [m] maximum trajectory arc length that is checked for out_of_lane collisions      |

| Parameter /threshold | Type   | Description                                                      |
| -------------------- | ------ | ---------------------------------------------------------------- |
| `time_threshold`     | double | [s] consider objects that will reach an overlap within this time |

| Parameter /ttc | Type   | Description                                                                                            |
| -------------- | ------ | ------------------------------------------------------------------------------------------------------ |
| `threshold`    | double | [s] consider objects with an estimated time to collision bellow this value while ego is on the overlap |

| Parameter /objects                      | Type   | Description                                                                     |
| --------------------------------------- | ------ | ------------------------------------------------------------------------------- |
| `minimum_velocity`                      | double | [m/s] ignore objects with a velocity lower than this value                      |
| `predicted_path_min_confidence`         | double | [-] minimum confidence required for a predicted path to be considered           |
| `cut_predicted_paths_beyond_red_lights` | bool   | [-] if true, predicted paths are cut beyond the stop line of red traffic lights |
| `ignore_behind_ego`                     | bool   | [-] if true, objects behind the ego vehicle are ignored                         |

| Parameter /action              | Type   | Description                                                           |
| ------------------------------ | ------ | --------------------------------------------------------------------- |
| `precision`                    | double | [m] precision when inserting a stop pose in the trajectory            |
| `longitudinal_distance_buffer` | double | [m] safety distance buffer to keep in front of the ego vehicle        |
| `lateral_distance_buffer`      | double | [m] safety distance buffer to keep on the side of the ego vehicle     |
| `min_duration`                 | double | [s] minimum duration needed before a decision can be canceled         |
| `slowdown.distance_threshold`  | double | [m] insert a slow down when closer than this distance from an overlap |
| `slowdown.velocity`            | double | [m] slow down velocity                                                |
| `stop.distance_threshold`      | double | [m] insert a stop when closer than this distance from an overlap      |

| Parameter /ego       | Type   | Description                                          |
| -------------------- | ------ | ---------------------------------------------------- |
| `extra_front_offset` | double | [m] extra front distance to add to the ego footprint |
| `extra_rear_offset`  | double | [m] extra rear distance to add to the ego footprint  |
| `extra_left_offset`  | double | [m] extra left distance to add to the ego footprint  |
| `extra_right_offset` | double | [m] extra right distance to add to the ego footprint |
