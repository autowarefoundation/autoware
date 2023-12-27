# map_based_prediction

## Role

`map_based_prediction` is a module to predict the future paths (and their probabilities) of other vehicles and pedestrians according to the shape of the map and the surrounding environment.

## Assumptions

- The following information about the target obstacle is needed
  - Label (type of person, car, etc.)
  - The object position in the current time and predicted position in the future time.
- The following information about the surrounding environment is needed
  - Road network information with Lanelet2 format

## Inner-workings / Algorithms

### Flow chart

<div align="center">
  <img src="media/map_based_prediction_flow.drawio.svg" width=20%>
</div>

### Path prediction for road users

#### Remove old object history

Store time-series data of objects to determine the vehicle's route and to detect lane change for several duration. Object Data contains the object's position, speed, and time information.

#### Get current lanelet and update Object history

Search one or more lanelets satisfying the following conditions for each target object and store them in the ObjectData.

- The CoG of the object must be inside the lanelet.
- The centerline of the lanelet must have two or more points.
- The angle difference between the lanelet and the direction of the object must be within the threshold given by the parameters.
  - The angle flip is allowed, the condition is `diff_yaw < threshold or diff_yaw > pi - threshold`.
- The lanelet must be reachable from the lanelet recorded in the past history.

#### Get predicted reference path

- Get reference path:
  - Create a reference path for the object from the associated lanelet.
- Predict object maneuver:
  - Generate predicted paths for the object.
  - Assign probability to each maneuver of `Lane Follow`, `Left Lane Change`, and `Right Lane Change` based on the object history and the reference path obtained in the first step.
  - Lane change decision is based on two domains:
    - Geometric domain: the lateral distance between the center of gravity of the object and left/right boundaries of the lane.
    - Time domain: estimated time margin for the object to reach the left/right bound.

The conditions for left lane change detection are:

- Check if the distance to the left lane boundary is less than the distance to the right lane boundary.
- Check if the distance to the left lane boundary is less than a `dist_threshold_to_bound_`.
- Check if the lateral velocity direction is towards the left lane boundary.
- Check if the time to reach the left lane boundary is less than `time_threshold_to_bound_`.

Lane change logics is illustrated in the figure below.An example of how to tune the parameters is described later.

![lane change detection](./media/lane_change_detection.drawio.svg)

- Calculate object probability:
  - The path probability obtained above is calculated based on the current position and angle of the object.
- Refine predicted paths for smooth movement:
  - The generated predicted paths are recomputed to take the vehicle dynamics into account.
  - The path is calculated with minimum jerk trajectory implemented by 4th/5th order spline for lateral/longitudinal motion.

### Tuning lane change detection logic

Currently we provide three parameters to tune lane change detection:

- `dist_threshold_to_bound_`: maximum distance from lane boundary allowed for lane changing vehicle
- `time_threshold_to_bound_`: maximum time allowed for lane change vehicle to reach the boundary
- `cutoff_freq_of_velocity_lpf_`: cutoff frequency of low pass filter for lateral velocity

You can change these parameters in rosparam in the table below.

| param name                                          | default value |
| --------------------------------------------------- | ------------- |
| `dist_threshold_for_lane_change_detection`          | `1.0` [m]     |
| `time_threshold_for_lane_change_detection`          | `5.0` [s]     |
| `cutoff_freq_of_velocity_for_lane_change_detection` | `0.1` [Hz]    |

#### Tuning threshold parameters

Increasing these two parameters will slow down and stabilize the lane change estimation.

Normally, we recommend tuning only `time_threshold_for_lane_change_detection` because it is the more important factor for lane change decision.

#### Tuning lateral velocity calculation

Lateral velocity calculation is also a very important factor for lane change decision because it is used in the time domain decision.

The predicted time to reach the lane boundary is calculated by

$$
t_{predicted} = \dfrac{d_{lat}}{v_{lat}}
$$

where $d_{lat}$ and $v_{lat}$ represent the lateral distance to the lane boundary and the lateral velocity, respectively.

Lowering the cutoff frequency of the low-pass filter for lateral velocity will make the lane change decision more stable but slower. Our setting is very conservative, so you may increase this parameter if you want to make the lane change decision faster.

For the additional information, here we show how we calculate lateral velocity.

| lateral velocity calculation method                           | equation                           | description                                                                                                                                                                                                                       |
| ------------------------------------------------------------- | ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [**applied**] time derivative of lateral distance             | $\dfrac{\Delta d_{lat}}{\Delta t}$ | Currently, we use this method to deal with winding roads. Since this time differentiation easily becomes noisy, we also use a low-pass filter to get smoothed velocity.                                                           |
| [not applied] Object Velocity Projection to Lateral Direction | $v_{obj} \sin(\theta)$             | Normally, object velocities are less noisy than the time derivative of lateral distance. But the yaw difference $\theta$ between the lane and object directions sometimes becomes discontinuous, so we did not adopt this method. |

Currently, we use the upper method with a low-pass filter to calculate lateral velocity.

### Path generation

Path generation is generated on the frenet frame. The path is generated by the following steps:

1. Get the frenet frame of the reference path.
2. Generate the frenet frame of the current position of the object and the finite position of the object.
3. Optimize the path in each longitudinal and lateral coordinate of the frenet frame. (Use the quintic polynomial to fit start and end conditions.)
4. Convert the path to the global coordinate.

See paper [2] for more details.

#### Tuning lateral path shape

`lateral_control_time_horizon` parameter supports the tuning of the lateral path shape. This parameter is used to calculate the time to reach the reference path. The smaller the value, the more the path will be generated to reach the reference path quickly. (Mostly the center of the lane.)

#### Pruning predicted paths with lateral acceleration constraint

It is possible to apply a maximum lateral acceleration constraint to generated vehicle paths. This check verifies if it is possible for the vehicle to perform the predicted path without surpassing a lateral acceleration threshold `max_lateral_accel` when taking a curve. If it is not possible, it checks if the vehicle can slow down on time to take the curve with a deceleration of `min_acceleration_before_curve` and comply with the constraint. If that is also not possible, the path is eliminated.

Currently we provide three parameters to tune the lateral acceleration constraint:

- `check_lateral_acceleration_constraints_`: to enable the constraint check.
- `max_lateral_accel_`: max acceptable lateral acceleration for predicted paths (absolute value).
- `min_acceleration_before_curve_`: the minimum acceleration the vehicle would theoretically use to slow down before a curve is taken (must be negative).

You can change these parameters in rosparam in the table below.

| param name                                | default value  |
| ----------------------------------------- | -------------- |
| `check_lateral_acceleration_constraints_` | `false` [bool] |
| `max_lateral_accel`                       | `2.0` [m/s^2]  |
| `min_acceleration_before_curve`           | `-2.0` [m/s^2] |

### Path prediction for crosswalk users

This module treats **Pedestrians** and **Bicycles** as objects using the crosswalk, and outputs prediction path based on map and estimated object's velocity, assuming the object has intention to cross the crosswalk, if the objects satisfies at least one of the following conditions:

- move toward the crosswalk
- stop near the crosswalk

<div align="center">
  <img src="images/target_objects.svg" width=90%>
</div>

If there are a reachable crosswalk entry points within the `prediction_time_horizon` and the objects satisfies above condition, this module outputs additional predicted path to cross the opposite side via the crosswalk entry point.

<div align="center">
  <img src="images/outside_road.svg" width=90%>
</div>

If the target object is inside the road or crosswalk, this module outputs one or two additional prediction path(s) to reach exit point of the crosswalk. The number of prediction paths are depend on whether object is moving or not. If the object is moving, this module outputs one prediction path toward an exit point that existed in the direction of object's movement. One the other hand, if the object has stopped, it is impossible to infer which exit points the object want to go, so this module outputs two prediction paths toward both side exit point.

<div align="center">
  <img src="images/inside_road.svg" width=90%>
</div>

## Inputs / Outputs

### Input

| Name                                               | Type                                                 | Description                              |
| -------------------------------------------------- | ---------------------------------------------------- | ---------------------------------------- |
| `~/perception/object_recognition/tracking/objects` | `autoware_auto_perception_msgs::msg::TrackedObjects` | tracking objects without predicted path. |
| `~/vector_map`                                     | `autoware_auto_mapping_msgs::msg::HADMapBin`         | binary data of Lanelet2 Map.             |

### Output

| Name                     | Type                                                   | Description                                                                           |
| ------------------------ | ------------------------------------------------------ | ------------------------------------------------------------------------------------- |
| `~/input/objects`        | `autoware_auto_perception_msgs::msg::TrackedObjects`   | tracking objects. Default is set to `/perception/object_recognition/tracking/objects` |
| `~/output/objects`       | `autoware_auto_perception_msgs::msg::PredictedObjects` | tracking objects with predicted path.                                                 |
| `~/objects_path_markers` | `visualization_msgs::msg::MarkerArray`                 | marker for visualization.                                                             |

## Parameters

| Parameter                                                        | Unit  | Type   | Description                                                                                                                           |
| ---------------------------------------------------------------- | ----- | ------ | ------------------------------------------------------------------------------------------------------------------------------------- |
| `enable_delay_compensation`                                      | [-]   | bool   | flag to enable the time delay compensation for the position of the object                                                             |
| `prediction_time_horizon`                                        | [s]   | double | predict time duration for predicted path                                                                                              |
| `lateral_control_time_horizon`                                   | [s]   | double | time duration for predicted path will reach the reference path (mostly center of the lane)                                            |
| `prediction_sampling_delta_time`                                 | [s]   | double | sampling time for points in predicted path                                                                                            |
| `min_velocity_for_map_based_prediction`                          | [m/s] | double | apply map-based prediction to the objects with higher velocity than this value                                                        |
| `min_crosswalk_user_velocity`                                    | [m/s] | double | minimum velocity used when crosswalk user's velocity is calculated                                                                    |
| `max_crosswalk_user_delta_yaw_threshold_for_lanelet`             | [rad] | double | maximum yaw difference between crosswalk user and lanelet to use in path prediction for crosswalk users                               |
| `dist_threshold_for_searching_lanelet`                           | [m]   | double | The threshold of the angle used when searching for the lane to which the object belongs                                               |
| `delta_yaw_threshold_for_searching_lanelet`                      | [rad] | double | The threshold of the angle used when searching for the lane to which the object belongs                                               |
| `sigma_lateral_offset`                                           | [m]   | double | Standard deviation for lateral position of objects                                                                                    |
| `sigma_yaw_angle_deg`                                            | [deg] | double | Standard deviation yaw angle of objects                                                                                               |
| `object_buffer_time_length`                                      | [s]   | double | Time span of object history to store the information                                                                                  |
| `history_time_length`                                            | [s]   | double | Time span of object information used for prediction                                                                                   |
| `prediction_time_horizon_rate_for_validate_shoulder_lane_length` | [-]   | double | prediction path will disabled when the estimated path length exceeds lanelet length. This parameter control the estimated path length |

## Assumptions / Known limits

- For object types of passenger car, bus, and truck
  - The predicted path of the object follows the road structure.
  - For the object not being on any roads, the predicted path is generated by just a straight line prediction.
  - For the object on a lanelet but moving in a different direction of the road, the predicted path is just straight.
  - Vehicle dynamics may not be properly considered in the predicted path.
- For object types of person and motorcycle
  - The predicted path is generated by just a straight line in all situations except for "around crosswalk".
- For all obstacles
  - In the prediction, the vehicle motion is assumed to be a constant velocity due to a lack of acceleration information.

## Reference

1. M. Werling, J. Ziegler, S. Kammel, and S. Thrun, “Optimal trajectory generation for dynamic street scenario in a frenet frame,” IEEE International Conference on Robotics and Automation, Anchorage, Alaska, USA, May 2010.
2. A. Houenou, P. Bonnifait, V. Cherfaoui, and Wen Yao, “Vehicle trajectory prediction based on motion model and maneuver recognition,” in 2013 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, nov 2013, pp. 4363-4369.
