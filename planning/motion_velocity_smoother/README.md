# Motion Velocity Smoother

## Purpose

`motion_velocity_smoother` outputs a desired velocity profile on a reference trajectory.
This module plans a velocity profile within the limitations of the velocity, the acceleration and the jerk to realize both the maximization of velocity and the ride quality.
We call this module `motion_velocity_smoother` because the limitations of the acceleration and the jerk means the smoothness of the velocity profile.

## Inner-workings / Algorithms

### Flow chart

![motion_velocity_smoother_flow](./media/motion_velocity_smoother_flow.drawio.svg)

#### Extract trajectory

For the point on the reference trajectory closest to the center of the rear wheel axle of the vehicle, it extracts the reference path between `extract_behind_dist` behind and `extract_ahead_dist` ahead.

#### Apply external velocity limit

It applies the velocity limit input from the external of `motion_velocity_smoother`.
Remark that the external velocity limit is different from the velocity limit already set on the map and the reference trajectory.
The external velocity is applied at the position that it is able to reach the velocity limit with the deceleration and the jerk constraints set as the parameter.

#### Apply stop approaching velocity

It applies the velocity limit near the stopping point.
This function is used to approach near the obstacle or improve the accuracy of stopping.

#### Apply lateral acceleration limit

It applies the velocity limit to decelerate at the curve.
It calculates the velocity limit from the curvature of the reference trajectory and the maximum lateral acceleration `max_lateral_accel`.
The velocity limit is set as not to fall under `min_curve_velocity`.

Note: velocity limit that requests larger than `nominal.jerk` is not applied. In other words, even if a sharp curve is planned just in front of the ego, no deceleration is performed.

#### Apply steering rate limit

It calculates the desired steering angles of trajectory points. and it applies the steering rate limit. If the (`steering_angle_rate` > `max_steering_angle_rate`), it decreases the velocity of the trajectory point to acceptable velocity.

#### Resample trajectory

It resamples the points on the reference trajectory with designated time interval.
Note that the range of the length of the trajectory is set between `min_trajectory_length` and `max_trajectory_length`, and the distance between two points is longer than `min_trajectory_interval_distance`.
It samples densely up to the distance traveled between `resample_time` with the current velocity, then samples sparsely after that.
By sampling according to the velocity, both calculation load and accuracy are achieved since it samples finely at low velocity and coarsely at high velocity.

#### Calculate initial state

Calculate initial values for velocity planning.
The initial values are calculated according to the situation as shown in the following table.

| Situation                                                     | Initial velocity       | Initial acceleration   |
| ------------------------------------------------------------- | ---------------------- | ---------------------- |
| First calculation                                             | Current velocity       | 0.0                    |
| Engaging                                                      | `engage_velocity`      | `engage_acceleration`  |
| Deviate between the planned velocity and the current velocity | Current velocity       | Previous planned value |
| Normal                                                        | Previous planned value | Previous planned value |

#### Smooth velocity

It plans the velocity.
The algorithm of velocity planning is chosen from `JerkFiltered`, `L2` and `Linf`, and it is set in the launch file.
In these algorithms, they use OSQP[1] as the solver of the optimization.

##### JerkFiltered

It minimizes the sum of the minus of the square of the velocity and the square of the violation of the velocity limit, the acceleration limit and the jerk limit.

##### L2

It minimizes the sum of the minus of the square of the velocity, the square of the the pseudo-jerk[2] and the square of the violation of the velocity limit and the acceleration limit.

##### Linf

It minimizes the sum of the minus of the square of the velocity, the maximum absolute value of the the pseudo-jerk[2] and the square of the violation of the velocity limit and the acceleration limit.

#### Post process

It performs the post-process of the planned velocity.

- Set zero velocity ahead of the stopping point
- Set maximum velocity given in the config named `max_velocity`
- Set velocity behind the current pose
- Resample trajectory (`post resampling`)
- Output debug data

After the optimization, a resampling called `post resampling` is performed before passing the optimized trajectory to the next node. Since the required path interval from optimization may be different from the one for the next module, `post resampling` helps to fill this gap. Therefore, in `post resampling`, it is necessary to check the path specification of the following module to determine the parameters. Note that if the computational load of the optimization algorithm is high and the path interval is sparser than the path specification of the following module in the first resampling, `post resampling` would resample the trajectory densely. On the other hand, if the computational load of the optimization algorithm is small and the path interval is denser than the path specification of the following module in the first resampling, the path is sparsely resampled according to the specification of the following module.

## Inputs / Outputs

### Input

| Name                                       | Type                                     | Description                   |
| ------------------------------------------ | ---------------------------------------- | ----------------------------- |
| `~/input/trajectory`                       | `autoware_auto_planning_msgs/Trajectory` | Reference trajectory          |
| `/planning/scenario_planning/max_velocity` | `std_msgs/Float32`                       | External velocity limit [m/s] |
| `/localization/kinematic_state`            | `nav_msgs/Odometry`                      | Current odometry              |
| `/tf`                                      | `tf2_msgs/TFMessage`                     | TF                            |
| `/tf_static`                               | `tf2_msgs/TFMessage`                     | TF static                     |

### Output

| Name                                               | Type                                     | Description                                                                                               |
| -------------------------------------------------- | ---------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `~/output/trajectory`                              | `autoware_auto_planning_msgs/Trajectory` | Modified trajectory                                                                                       |
| `/planning/scenario_planning/current_max_velocity` | `std_msgs/Float32`                       | Current external velocity limit [m/s]                                                                     |
| `~/closest_velocity`                               | `std_msgs/Float32`                       | Planned velocity closest to ego base_link (for debug)                                                     |
| `~/closest_acceleration`                           | `std_msgs/Float32`                       | Planned acceleration closest to ego base_link (for debug)                                                 |
| `~/closest_jerk`                                   | `std_msgs/Float32`                       | Planned jerk closest to ego base_link (for debug)                                                         |
| `~/debug/trajectory_raw`                           | `autoware_auto_planning_msgs/Trajectory` | Extracted trajectory (for debug)                                                                          |
| `~/debug/trajectory_external_velocity_limited`     | `autoware_auto_planning_msgs/Trajectory` | External velocity limited trajectory (for debug)                                                          |
| `~/debug/trajectory_lateral_acc_filtered`          | `autoware_auto_planning_msgs/Trajectory` | Lateral acceleration limit filtered trajectory (for debug)                                                |
| `~/debug/trajectory_steering_rate_limited`         | `autoware_auto_planning_msgs/Trajectory` | Steering angle rate limit filtered trajectory (for debug)                                                 |
| `~/debug/trajectory_time_resampled`                | `autoware_auto_planning_msgs/Trajectory` | Time resampled trajectory (for debug)                                                                     |
| `~/distance_to_stopline`                           | `std_msgs/Float32`                       | Distance to stop line from current ego pose (max 50 m) (for debug)                                        |
| `~/stop_speed_exceeded`                            | `std_msgs/Bool`                          | It publishes `true` if planned velocity on the point which the maximum velocity is zero is over threshold |

## Parameters

### Constraint parameters

| Name           | Type     | Description                                    | Default value |
| :------------- | :------- | :--------------------------------------------- | :------------ |
| `max_velocity` | `double` | Max velocity limit [m/s]                       | 20.0          |
| `max_accel`    | `double` | Max acceleration limit [m/ss]                  | 1.0           |
| `min_decel`    | `double` | Min deceleration limit [m/ss]                  | -0.5          |
| `stop_decel`   | `double` | Stop deceleration value at a stop point [m/ss] | 0.0           |
| `max_jerk`     | `double` | Max jerk limit [m/sss]                         | 1.0           |
| `min_jerk`     | `double` | Min jerk limit [m/sss]                         | -0.5          |

### External velocity limit parameter

| Name                                       | Type     | Description                                           | Default value |
| :----------------------------------------- | :------- | :---------------------------------------------------- | :------------ |
| `margin_to_insert_external_velocity_limit` | `double` | margin distance to insert external velocity limit [m] | 0.3           |

### Curve parameters

| Name                                   | Type     | Description                                                                                                                                                                                                  | Default value |
| :------------------------------------- | :------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `max_lateral_accel`                    | `double` | Max lateral acceleration limit [m/ss]                                                                                                                                                                        | 0.5           |
| `min_curve_velocity`                   | `double` | Min velocity at lateral acceleration limit [m/ss]                                                                                                                                                            | 2.74          |
| `decel_distance_before_curve`          | `double` | Distance to slowdown before a curve for lateral acceleration limit [m]                                                                                                                                       | 3.5           |
| `decel_distance_after_curve`           | `double` | Distance to slowdown after a curve for lateral acceleration limit [m]                                                                                                                                        | 2.0           |
| `min_decel_for_lateral_acc_lim_filter` | `double` | Deceleration limit to avoid sudden braking by the lateral acceleration filter [m/ss]. Strong limitation degrades the deceleration response to the appearance of sharp curves due to obstacle avoidance, etc. | -2.5          |

### Engage & replan parameters

| Name                           | Type     | Description                                                                                                                        | Default value |
| :----------------------------- | :------- | :--------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `replan_vel_deviation`         | `double` | Velocity deviation to replan initial velocity [m/s]                                                                                | 5.53          |
| `engage_velocity`              | `double` | Engage velocity threshold [m/s] (if the trajectory velocity is higher than this value, use this velocity for engage vehicle speed) | 0.25          |
| `engage_acceleration`          | `double` | Engage acceleration [m/ss] (use this acceleration when engagement)                                                                 | 0.1           |
| `engage_exit_ratio`            | `double` | Exit engage sequence to normal velocity planning when the velocity exceeds engage_exit_ratio x engage_velocity.                    | 0.5           |
| `stop_dist_to_prohibit_engage` | `double` | If the stop point is in this distance, the speed is set to 0 not to move the vehicle [m]                                           | 0.5           |

### Stopping velocity parameters

| Name                | Type     | Description                                                                           | Default value |
| :------------------ | :------- | :------------------------------------------------------------------------------------ | :------------ |
| `stopping_velocity` | `double` | change target velocity to this value before v=0 point [m/s]                           | 2.778         |
| `stopping_distance` | `double` | distance for the stopping_velocity [m]. 0 means the stopping velocity is not applied. | 0.0           |

### Extraction parameters

| Name                  | Type     | Description                                                     | Default value |
| :-------------------- | :------- | :-------------------------------------------------------------- | :------------ |
| `extract_ahead_dist`  | `double` | Forward trajectory distance used for planning [m]               | 200.0         |
| `extract_behind_dist` | `double` | backward trajectory distance used for planning [m]              | 5.0           |
| `delta_yaw_threshold` | `double` | Allowed delta yaw between ego pose and trajectory pose [radian] | 1.0472        |

### Resampling parameters

| Name                           | Type     | Description                                            | Default value |
| :----------------------------- | :------- | :----------------------------------------------------- | :------------ |
| `max_trajectory_length`        | `double` | Max trajectory length for resampling [m]               | 200.0         |
| `min_trajectory_length`        | `double` | Min trajectory length for resampling [m]               | 30.0          |
| `resample_time`                | `double` | Resample total time [s]                                | 10.0          |
| `dense_dt`                     | `double` | resample time interval for dense sampling [s]          | 0.1           |
| `dense_min_interval_distance`  | `double` | minimum points-interval length for dense sampling [m]  | 0.1           |
| `sparse_dt`                    | `double` | resample time interval for sparse sampling [s]         | 0.5           |
| `sparse_min_interval_distance` | `double` | minimum points-interval length for sparse sampling [m] | 4.0           |

### Resampling parameters for post process

| Name                                | Type     | Description                                            | Default value |
| :---------------------------------- | :------- | :----------------------------------------------------- | :------------ |
| `post_max_trajectory_length`        | `double` | max trajectory length for resampling [m]               | 300.0         |
| `post_min_trajectory_length`        | `double` | min trajectory length for resampling [m]               | 30.0          |
| `post_resample_time`                | `double` | resample total time for dense sampling [s]             | 10.0          |
| `post_dense_dt`                     | `double` | resample time interval for dense sampling [s]          | 0.1           |
| `post_dense_min_interval_distance`  | `double` | minimum points-interval length for dense sampling [m]  | 0.1           |
| `post_sparse_dt`                    | `double` | resample time interval for sparse sampling [s]         | 0.1           |
| `post_sparse_min_interval_distance` | `double` | minimum points-interval length for sparse sampling [m] | 1.0           |

### Limit steering angle rate parameters

| Name                             | Type     | Description                                                              | Default value |
| :------------------------------- | :------- | :----------------------------------------------------------------------- | :------------ |
| `max_steering_angle_rate`        | `double` | Maximum steering angle rate [degree/s]                                   | 40.0          |
| `resample_ds`                    | `double` | Distance between trajectory points [m]                                   | 0.1           |
| `curvature_threshold`            | `double` | If curvature > curvature_threshold, steeringRateLimit is triggered [1/m] | 0.02          |
| `curvature_calculation_distance` | `double` | Distance of points while curvature is calculating [m]                    | 1.0           |

### Weights for optimization

#### JerkFiltered

| Name            | Type     | Description                           | Default value |
| :-------------- | :------- | :------------------------------------ | :------------ |
| `jerk_weight`   | `double` | Weight for "smoothness" cost for jerk | 10.0          |
| `over_v_weight` | `double` | Weight for "over speed limit" cost    | 100000.0      |
| `over_a_weight` | `double` | Weight for "over accel limit" cost    | 5000.0        |
| `over_j_weight` | `double` | Weight for "over jerk limit" cost     | 1000.0        |

#### L2

| Name                 | Type     | Description                        | Default value |
| :------------------- | :------- | :--------------------------------- | :------------ |
| `pseudo_jerk_weight` | `double` | Weight for "smoothness" cost       | 100.0         |
| `over_v_weight`      | `double` | Weight for "over speed limit" cost | 100000.0      |
| `over_a_weight`      | `double` | Weight for "over accel limit" cost | 1000.0        |

#### Linf

| Name                 | Type     | Description                        | Default value |
| :------------------- | :------- | :--------------------------------- | :------------ |
| `pseudo_jerk_weight` | `double` | Weight for "smoothness" cost       | 100.0         |
| `over_v_weight`      | `double` | Weight for "over speed limit" cost | 100000.0      |
| `over_a_weight`      | `double` | Weight for "over accel limit" cost | 1000.0        |

### Others

| Name                          | Type     | Description                                                                                       | Default value |
| :---------------------------- | :------- | :------------------------------------------------------------------------------------------------ | :------------ |
| `over_stop_velocity_warn_thr` | `double` | Threshold to judge that the optimized velocity exceeds the input velocity on the stop point [m/s] | 1.389         |

<!-- Write parameters of this package.

Example:
  ### Node Parameters

  | Name                   | Type | Description                     |
  | ---------------------- | ---- | ------------------------------- |
  | `output_debug_markers` | bool | whether to output debug markers |

  ### Core Parameters

  | Name                 | Type     | Description                                                          |
  | -------------------- | -------- | -------------------------------------------------------------------- |
  | `min_object_size_m`  | `double` | minimum object size to be selected as avoidance target obstacles [m] |
  | `avoidance_margin_m` | `double` | avoidance margin to obstacles [m]                                    |
-->

## Assumptions / Known limits

- Assume that the velocity limit or the stopping point is properly set at the point on the reference trajectory
- If the velocity limit set in the reference path cannot be achieved by the designated constraints of the deceleration and the jerk, decelerate while suppressing the velocity, the acceleration and the jerk deviation as much as possible
- The importance of the deviations is set in the config file

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

[1] B. Stellato, et al., "OSQP: an operator splitting solver for quadratic programs", Mathematical Programming Computation, 2020, [10.1007/s12532-020-00179-2](https://link.springer.com/article/10.1007/s12532-020-00179-2).

[2] Y. Zhang, et al., "Toward a More Complete, Flexible, and Safer Speed Planning for Autonomous Driving via Convex Optimization", Sensors, vol. 18, no. 7, p. 2185, 2018, [10.3390/s18072185](https://doi.org/10.3390/s18072185)

## (Optional) Future extensions / Unimplemented parts
