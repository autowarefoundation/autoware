# MPC Lateral Controller

This is the design document for the lateral controller node
in the `trajectory_follower_node` package.

## Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This node is used to general lateral control commands (steering angle and steering rate)
when following a path.

## Design

<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The node uses an implementation of linear model predictive control (MPC) for accurate path tracking.
The MPC uses a model of the vehicle to simulate the trajectory resulting from the control command.
The optimization of the control command is formulated as a Quadratic Program (QP).

Different vehicle models are implemented:

- kinematics : bicycle kinematics model with steering 1st-order delay.
- kinematics_no_delay : bicycle kinematics model without steering delay.
- dynamics : bicycle dynamics model considering slip angle.
  The kinematics model is being used by default. Please see the reference [1] for more details.

For the optimization, a Quadratic Programming (QP) solver is used and two options are currently implemented:

<!-- cspell: ignore ADMM -->

- unconstraint_fast : use least square method to solve unconstraint QP with eigen.
- [osqp](https://osqp.org/): run the [following ADMM](https://web.stanford.edu/~boyd/papers/admm_distr_stats.html)
  algorithm (for more details see the related papers at
  the [Citing OSQP](https://web.stanford.edu/~boyd/papers/admm_distr_stats.html) section):

### Filtering

Filtering is required for good noise reduction.
A [Butterworth filter](https://en.wikipedia.org/wiki/Butterworth_filter) is employed for processing the yaw and lateral errors, which are used as inputs for the MPC, as well as for refining the output steering angle.
Other filtering methods can be considered as long as the noise reduction performances are good
enough.
The moving average filter for example is not suited and can yield worse results than without any
filtering.

## Assumptions / Known limits

<!-- Required -->

The tracking is not accurate if the first point of the reference trajectory is at or in front of the current ego pose.

## Inputs / Outputs / API

<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Inputs

Set the following from the [controller_node](../trajectory_follower_node/README.md)

- `autoware_auto_planning_msgs/Trajectory` : reference trajectory to follow.
- `nav_msgs/Odometry`: current odometry
- `autoware_auto_vehicle_msgs/SteeringReport`: current steering

### Outputs

Return LateralOutput which contains the following to the controller node

- `autoware_auto_control_msgs/AckermannLateralCommand`
- LateralSyncData
  - steer angle convergence

### MPC class

The `MPC` class (defined in `mpc.hpp`) provides the interface with the MPC algorithm.
Once a vehicle model, a QP solver, and the reference trajectory to follow have been set
(using `setVehicleModel()`, `setQPSolver()`, `setReferenceTrajectory()`), a lateral control command
can be calculated by providing the current steer, velocity, and pose to function `calculateMPC()`.

### Parameter description

The default parameters defined in `param/lateral_controller_defaults.param.yaml` are adjusted to the
AutonomouStuff Lexus RX 450h for under 40 km/h driving.

#### System

| Name                      | Type    | Description                                                                 | Default value |
| :------------------------ | :------ | :-------------------------------------------------------------------------- | :------------ |
| traj_resample_dist        | double  | distance of waypoints in resampling [m]                                     | 0.1           |
| use_steer_prediction      | boolean | flag for using steer prediction (do not use steer measurement)              | false         |
| admissible_position_error | double  | stop vehicle when following position error is larger than this value [m]    | 5.0           |
| admissible_yaw_error_rad  | double  | stop vehicle when following yaw angle error is larger than this value [rad] | 1.57          |

#### Path Smoothing

| Name                              | Type    | Description                                                                                                                                          | Default value |
| :-------------------------------- | :------ | :--------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| enable_path_smoothing             | boolean | path smoothing flag. This should be true when uses path resampling to reduce resampling noise.                                                       | false         |
| path_filter_moving_ave_num        | int     | number of data points moving average filter for path smoothing                                                                                       | 25            |
| curvature_smoothing_num_traj      | int     | index distance of points used in curvature calculation for trajectory: p(i-num), p(i), p(i+num). larger num makes less noisy values.                 | 15            |
| curvature_smoothing_num_ref_steer | int     | index distance of points used in curvature calculation for reference steering command: p(i-num), p(i), p(i+num). larger num makes less noisy values. | 15            |

#### Trajectory Extending

| Name                                  | Type    | Description                                   | Default value |
| :------------------------------------ | :------ | :-------------------------------------------- | :------------ |
| extend_trajectory_for_end_yaw_control | boolean | trajectory extending flag for end yaw control | true          |

#### MPC Optimization

| Name                                                | Type   | Description                                                                                                  | Default value |
| :-------------------------------------------------- | :----- | :----------------------------------------------------------------------------------------------------------- | :------------ |
| qp_solver_type                                      | string | QP solver option. described below in detail.                                                                 | "osqp"        |
| mpc_prediction_horizon                              | int    | total prediction step for MPC                                                                                | 50            |
| mpc_prediction_dt                                   | double | prediction period for one step [s]                                                                           | 0.1           |
| mpc_weight_lat_error                                | double | weight for lateral error                                                                                     | 1.0           |
| mpc_weight_heading_error                            | double | weight for heading error                                                                                     | 0.0           |
| mpc_weight_heading_error_squared_vel                | double | weight for heading error \* velocity                                                                         | 0.3           |
| mpc_weight_steering_input                           | double | weight for steering error (steer command - reference steer)                                                  | 1.0           |
| mpc_weight_steering_input_squared_vel               | double | weight for steering error (steer command - reference steer) \* velocity                                      | 0.25          |
| mpc_weight_lat_jerk                                 | double | weight for lateral jerk (steer(i) - steer(i-1)) \* velocity                                                  | 0.1           |
| mpc_weight_steer_rate                               | double | weight for steering rate [rad/s]                                                                             | 0.0           |
| mpc_weight_steer_acc                                | double | weight for derivatives of the steering rate [rad/ss]                                                         | 0.000001      |
| mpc_low_curvature_weight_lat_error                  | double | [used in a low curvature trajectory] weight for lateral error                                                | 0.1           |
| mpc_low_curvature_weight_heading_error              | double | [used in a low curvature trajectory] weight for heading error                                                | 0.0           |
| mpc_low_curvature_weight_heading_error_squared_vel  | double | [used in a low curvature trajectory] weight for heading error \* velocity                                    | 0.3           |
| mpc_low_curvature_weight_steering_input             | double | [used in a low curvature trajectory] weight for steering error (steer command - reference steer)             | 1.0           |
| mpc_low_curvature_weight_steering_input_squared_vel | double | [used in a low curvature trajectory] weight for steering error (steer command - reference steer) \* velocity | 0.25          |
| mpc_low_curvature_weight_lat_jerk                   | double | [used in a low curvature trajectory] weight for lateral jerk (steer(i) - steer(i-1)) \* velocity             | 0.0           |
| mpc_low_curvature_weight_steer_rate                 | double | [used in a low curvature trajectory] weight for steering rate [rad/s]                                        | 0.0           |
| mpc_low_curvature_weight_steer_acc                  | double | [used in a low curvature trajectory] weight for derivatives of the steering rate [rad/ss]                    | 0.000001      |
| mpc_low_curvature_thresh_curvature                  | double | threshold of curvature to use "low_curvature" parameter                                                      | 0.0           |
| mpc_weight_terminal_lat_error                       | double | terminal lateral error weight in matrix Q to improve mpc stability                                           | 1.0           |
| mpc_weight_terminal_heading_error                   | double | terminal heading error weight in matrix Q to improve mpc stability                                           | 0.1           |
| mpc_zero_ff_steer_deg                               | double | threshold that feed-forward angle becomes zero                                                               | 0.5           |
| mpc_acceleration_limit                              | double | limit on the vehicle's acceleration                                                                          | 2.0           |
| mpc_velocity_time_constant                          | double | time constant used for velocity smoothing                                                                    | 0.3           |
| mpc_min_prediction_length                           | double | minimum prediction length                                                                                    | 5.0           |

#### Vehicle Model

| Name                                 | Type     | Description                                                                        | Default value        |
| :----------------------------------- | :------- | :--------------------------------------------------------------------------------- | :------------------- |
| vehicle_model_type                   | string   | vehicle model type for mpc prediction                                              | "kinematics"         |
| input_delay                          | double   | steering input delay time for delay compensation                                   | 0.24                 |
| vehicle_model_steer_tau              | double   | steering dynamics time constant (1d approximation) [s]                             | 0.3                  |
| steer_rate_lim_dps_list_by_curvature | [double] | steering angle rate limit list depending on curvature [deg/s]                      | [40.0, 50.0, 60.0]   |
| curvature_list_for_steer_rate_lim    | [double] | curvature list for steering angle rate limit interpolation in ascending order [/m] | [0.001, 0.002, 0.01] |
| steer_rate_lim_dps_list_by_velocity  | [double] | steering angle rate limit list depending on velocity [deg/s]                       | [60.0, 50.0, 40.0]   |
| velocity_list_for_steer_rate_lim     | [double] | velocity list for steering angle rate limit interpolation in ascending order [m/s] | [10.0, 15.0, 20.0]   |
| acceleration_limit                   | double   | acceleration limit for trajectory velocity modification [m/ss]                     | 2.0                  |
| velocity_time_constant               | double   | velocity dynamics time constant for trajectory velocity modification [s]           | 0.3                  |

#### Lowpass Filter for Noise Reduction

| Name                      | Type   | Description                                                         | Default value |
| :------------------------ | :----- | :------------------------------------------------------------------ | :------------ |
| steering_lpf_cutoff_hz    | double | cutoff frequency of lowpass filter for steering output command [hz] | 3.0           |
| error_deriv_lpf_cutoff_hz | double | cutoff frequency of lowpass filter for error derivative [Hz]        | 5.0           |

#### Stop State

| Name                                         | Type    | Description                                                                                     | Default value |
| :------------------------------------------- | :------ | :---------------------------------------------------------------------------------------------- | :------------ |
| stop_state_entry_ego_speed <sup>\*1</sup>    | double  | threshold value of the ego vehicle speed used to the stop state entry condition                 | 0.001         |
| stop_state_entry_target_speed <sup>\*1</sup> | double  | threshold value of the target speed used to the stop state entry condition                      | 0.001         |
| converged_steer_rad                          | double  | threshold value of the steer convergence                                                        | 0.1           |
| keep_steer_control_until_converged           | boolean | keep steer control until steer is converged                                                     | true          |
| new_traj_duration_time                       | double  | threshold value of the time to be considered as new trajectory                                  | 1.0           |
| new_traj_end_dist                            | double  | threshold value of the distance between trajectory ends to be considered as new trajectory      | 0.3           |
| mpc_converged_threshold_rps                  | double  | threshold value to be sure output of the optimization is converged, it is used in stopped state | 0.01          |

(\*1) To prevent unnecessary steering movement, the steering command is fixed to the previous value in the stop state.

#### Steer Offset

Defined in the `steering_offset` namespace. This logic is designed as simple as possible, with minimum design parameters.

| Name                                | Type    | Description                                                                                      | Default value |
| :---------------------------------- | :------ | :----------------------------------------------------------------------------------------------- | :------------ |
| enable_auto_steering_offset_removal | boolean | Estimate the steering offset and apply compensation                                              | true          |
| update_vel_threshold                | double  | If the velocity is smaller than this value, the data is not used for the offset estimation       | 5.56          |
| update_steer_threshold              | double  | If the steering angle is larger than this value, the data is not used for the offset estimation. | 0.035         |
| average_num                         | int     | The average of this number of data is used as a steering offset.                                 | 1000          |
| steering_offset_limit               | double  | The angle limit to be applied to the offset compensation.                                        | 0.02          |

##### For dynamics model (WIP)

| Name          | Type   | Description                                 | Default value |
| :------------ | :----- | :------------------------------------------ | :------------ |
| cg_to_front_m | double | distance from baselink to the front axle[m] | 1.228         |
| cg_to_rear_m  | double | distance from baselink to the rear axle [m] | 1.5618        |
| mass_fl       | double | mass applied to front left tire [kg]        | 600           |
| mass_fr       | double | mass applied to front right tire [kg]       | 600           |
| mass_rl       | double | mass applied to rear left tire [kg]         | 600           |
| mass_rr       | double | mass applied to rear right tire [kg]        | 600           |
| cf            | double | front cornering power [N/rad]               | 155494.663    |
| cr            | double | rear cornering power [N/rad]                | 155494.663    |

### How to tune MPC parameters

#### Set kinematics information

First, it's important to set the appropriate parameters for vehicle kinematics. This includes parameters like `wheelbase`, which represents the distance between the front and rear wheels, and `max_steering_angle`, which indicates the maximum tire steering angle. These parameters should be set in the `vehicle_info.param.yaml`.

#### Set dynamics information

Next, you need to set the proper parameters for the dynamics model. These include the time constant `steering_tau` and time delay `steering_delay` for steering dynamics, and the maximum acceleration `mpc_acceleration_limit` and the time constant `mpc_velocity_time_constant` for velocity dynamics.

#### Confirmation of the input information

It's also important to make sure the input information is accurate. Information such as the velocity of the center of the rear wheel [m/s] and the steering angle of the tire [rad] is required. Please note that there have been frequent reports of performance degradation due to errors in input information. For instance, there are cases where the velocity of the vehicle is offset due to an unexpected difference in tire radius, or the tire angle cannot be accurately measured due to a deviation in the steering gear ratio or midpoint. It is suggested to compare information from multiple sensors (e.g., integrated vehicle speed and GNSS position, steering angle and IMU angular velocity), and ensure the input information for MPC is appropriate.

#### MPC weight tuning

Then, tune the weights of the MPC. One simple approach of tuning is to keep the weight for the lateral deviation (`weight_lat_error`) constant, and vary the input weight (`weight_steering_input`) while observing the trade-off between steering oscillation and control accuracy.

Here, `weight_lat_error` acts to suppress the lateral error in path following, while `weight_steering_input` works to adjust the steering angle to a standard value determined by the path's curvature. When `weight_lat_error` is large, the steering moves significantly to improve accuracy, which can cause oscillations. On the other hand, when `weight_steering_input` is large, the steering doesn't respond much to tracking errors, providing stable driving but potentially reducing tracking accuracy.

The steps are as follows:

1. Set `weight_lat_error` = 0.1, `weight_steering_input` = 1.0 and other weights to 0.
2. If the vehicle oscillates when driving, set `weight_steering_input` larger.
3. If the tracking accuracy is low, set `weight_steering_input` smaller.

If you want to adjust the effect only in the high-speed range, you can use `weight_steering_input_squared_vel`. This parameter corresponds to the steering weight in the high-speed range.

#### Descriptions for weights

- `weight_lat_error`: Reduce lateral tracking error. This acts like P gain in PID.
- `weight_heading_error`: Make a drive straight. This acts like D gain in PID.
- `weight_heading_error_squared_vel_coeff` : Make a drive straight in high speed range.
- `weight_steering_input`: Reduce oscillation of tracking.
- `weight_steering_input_squared_vel_coeff`: Reduce oscillation of tracking in high speed range.
- `weight_lat_jerk`: Reduce lateral jerk.
- `weight_terminal_lat_error`: Preferable to set a higher value than normal lateral weight `weight_lat_error` for stability.
- `weight_terminal_heading_error`: Preferable to set a higher value than normal heading weight `weight_heading_error` for stability.

#### Other tips for tuning

Here are some tips for adjusting other parameters:

- In theory, increasing terminal weights, `weight_terminal_lat_error` and `weight_terminal_heading_error`, can enhance the tracking stability. This method sometimes proves effective.
- A larger `prediction_horizon` and a smaller `prediction_sampling_time` are efficient for tracking performance. However, these come at the cost of higher computational costs.
- If you want to modify the weight according to the trajectory curvature (for instance, when you're driving on a sharp curve and want a larger weight), use `mpc_low_curvature_thresh_curvature` and adjust `mpc_low_curvature_weight_**` weights.
- If you want to adjust the steering rate limit based on the vehicle speed and trajectory curvature, you can modify the values of `steer_rate_lim_dps_list_by_curvature`, `curvature_list_for_steer_rate_lim`, `steer_rate_lim_dps_list_by_velocity`, `velocity_list_for_steer_rate_lim`. By doing this, you can enforce the steering rate limit during high-speed driving or relax it while curving.
- In case your target curvature appears jagged, adjusting `curvature_smoothing` becomes critically important for accurate curvature calculations. A larger value yields a smooth curvature calculation which reduces noise but can cause delay in feedforward computation and potentially degrade performance.
- Adjusting the `steering_lpf_cutoff_hz` value can also be effective to forcefully reduce computational noise. This refers to the cutoff frequency in the second order Butterworth filter installed in the final layer. The smaller the cutoff frequency, the stronger the noise reduction, but it also induce operation delay.
- If the vehicle consistently deviates laterally from the trajectory, it's most often due to the offset of the steering sensor or self-position estimation. It's preferable to eliminate these biases before inputting into MPC, but it's also possible to remove this bias within MPC. To utilize this, set `enable_auto_steering_offset_removal` to true and activate the steering offset remover. The steering offset estimation logic works when driving at high speeds with the steering close to the center, applying offset removal.
- If the onset of steering in curves is late, it's often due to incorrect delay time and time constant in the steering model. Please recheck the values of `input_delay` and `vehicle_model_steer_tau`. Additionally, as a part of its debug information, MPC outputs the current steering angle assumed by the MPC model, so please check if that steering angle matches the actual one.

## References / External links

<!-- Optional -->

- [1] Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking",
  Robotics Institute, Carnegie Mellon University, February 2009.

## Related issues

<!-- Required -->
