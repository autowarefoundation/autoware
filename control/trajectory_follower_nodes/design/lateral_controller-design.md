Lateral Controller {#lateral-controller-design}
=============================================

This is the design document for the lateral controller node
in the `trajectory_follower_nodes` package.

# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
This node is used to general lateral control commands (steering angle and steering rate)
when following a path.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
The node uses an implementation of linear model predictive control (MPC) for accurate path tracking.
The MPC uses a model of the vehicle to simulate the trajectory resulting from the control command.
The optimization of the control command is formulated as a Quadradic Program (QP).

These functionalities are implemented in the `trajectory_follower` package
(see @subpage trajectory_follower-mpc-design)

## Assumptions / Known limits
<!-- Required -->
The tracking is not accurate if the first point of the reference trajectory is at or in front of the current ego pose.
  - Issue to add points behind ego: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1273

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
Inputs
  - `input/reference_trajectory` : reference trajectory to follow.
  - `input/current_kinematic_state`: current state of the vehicle (position, velocity, etc).
Output
  - `output/lateral_control_cmd`: generated lateral control command.

## Parameter description

The default parameters defined in `param/lateral_controller_defaults.yaml` are adjusted to the
AutonomouStuff Lexus RX 450h for under 40 km/h driving.

| Name                              | Type   | Description                                                                                                                                       | Default value |
| :-------------------------------- | :----- | :------------------------------------------------------------------------------------------------------------------------------------------------ | :------------ |
| show_debug_info                   | bool   | display debug info                                                                                                                                | false         |
| ctrl_period                       | double | control period [s]                                                                                                                                | 0.03          |
| traj_resample_dist                | double | distance of waypoints in resampling [m]                                                                                                           | 0.1           |
| enable_path_smoothing             | bool   | path smoothing flag. This should be true when uses path resampling to reduce resampling noise.                                                    | true          |
| path_filter_moving_ave_num        | int    | number of data points moving average filter for path smoothing                                                                                    | 35            |
| path_smoothing_times              | int    | number of times of applying path smoothing filter                                                                                                 | 1             |
| curvature_smoothing_num_ref_steer | double | index distance of points used in curvature calculation for reference steer command: p(i-num), p(i), p(i+num). larger num makes less noisy values. | 35            |
| curvature_smoothing_num_traj      | double | index distance of points used in curvature calculation for trajectory: p(i-num), p(i), p(i+num). larger num makes less noisy values.              | 1             |
| steering_lpf_cutoff_hz            | double | cutoff frequency of lowpass filter for steering output command [hz]                                                                               | 3.0           |
| admissible_position_error         | double | stop vehicle when following position error is larger than this value [m].                                                                         | 5.0           |
| admissible_yaw_error_rad          | double | stop vehicle when following yaw angle error is larger than this value [rad].                                                                      | 1.57          |

### MPC algorithm

| Name                                    | Type   | Description                                                                                     | Default value     |
| :-------------------------------------- | :----- | :---------------------------------------------------------------------------------------------- | :---------------- |
| qp_solver_type                          | string | QP solver option. described below in detail.                                                    | unconstraint_fast |
| vehicle_model_type                      | string | vehicle model option. described below in detail.                                                | kinematics        |
| prediction_horizon                      | int    | total prediction step for MPC                                                                   | 70                |
| prediction_sampling_time                | double | prediction period for one step [s]                                                              | 0.1               |
| weight_lat_error                        | double | weight for lateral error                                                                        | 0.1               |
| weight_heading_error                    | double | weight for heading error                                                                        | 0.0               |
| weight_heading_error_squared_vel_coeff  | double | weight for heading error \* velocity                                                            | 5.0               |
| weight_steering_input                   | double | weight for steering error (steer command - reference steer)                                     | 1.0               |
| weight_steering_input_squared_vel_coeff | double | weight for steering error (steer command - reference steer) \* velocity                         | 0.25              |
| weight_lat_jerk                         | double | weight for lateral jerk (steer(i) - steer(i-1)) \* velocity                                     | 0.0               |
| weight_terminal_lat_error               | double | terminal cost weight for lateral error                                                          | 1.0               |
| weight_terminal_heading_error           | double | terminal cost weight for heading error                                                          | 0.1               |
| zero_ff_steer_deg                       | double | threshold of feedforward angle [deg]. feedforward angle smaller than this value is set to zero. | 2.0               |

### Vehicle

| Name          | Type   | Description                                                                        | Default value |
| :------------ | :----- | :--------------------------------------------------------------------------------- | :------------ |
| cg_to_front_m | double | distance from baselink to the front axle[m]                                        | 1.228         |
| cg_to_rear_m  | double | distance from baselink to the rear axle [m]                                        | 1.5618        |
| mass_fl       | double | mass applied to front left tire [kg]                                               | 600           |
| mass_fr       | double | mass applied to front right tire [kg]                                              | 600           |
| mass_rl       | double | mass applied to rear left tire [kg]                                                | 600           |
| mass_rr       | double | mass applied to rear right tire [kg]                                               | 600           |
| cf            | double | front cornering power [N/rad]                                                      | 155494.663    |
| cr            | double | rear cornering power [N/rad]                                                       | 155494.663    |
| steering_tau  | double | steering dynamics time constant (1d approximation) for vehicle model [s]           | 0.3           |
| steer_lim_deg | double | steering angle limit for vehicle model [deg]. This is also used for QP constraint. | 35.0          |

## How to tune MPC parameters

1. Set appropriate vehicle kinematics parameters for distance to front and rear axle, and `steer_lim_deg`.
Also check that the input `VehicleKinematicState` has appropriate values (speed: vehicle rear-wheels-center velocity [km/h], angle: steering (tire) angle [rad]).
These values give a vehicle information to the controller for path following.
Errors in these values cause fundamental tracking error.

2. Set appropriate vehicle dynamics parameters of `steering_tau`, which is the approximated delay from steering angle command to actual steering angle.

3. Set `weight_steering_input` = 1.0, `weight_lat_error` = 0.1, and other weights to 0.
If the vehicle oscillates when driving with low speed, set `weight_lat_error` smaller.

4. Adjust other weights.
One of the simple way for tuning is to increase `weight_lat_error` until oscillation occurs.
If the vehicle is unstable with very small `weight_lat_error`, increase terminal weight :
`weight_terminal_lat_error` and `weight_terminal_heading_error` to improve tracking stability.
Larger `prediction_horizon` and smaller `prediction_sampling_time` is effective for tracking performance, but it is a trade-off between computational costs.
Other parameters can be adjusted like below.

- `weight_lat_error`: Reduce lateral tracking error. This acts like P gain in PID.
- `weight_heading_error`: Make a drive straight. This acts like D gain in PID.
- `weight_heading_error_squared_vel_coeff` : Make a drive straight in high speed range.
- `weight_steering_input`: Reduce oscillation of tracking.
- `weight_steering_input_squared_vel_coeff`: Reduce oscillation of tracking in high speed range.
- `weight_lat_jerk`: Reduce lateral jerk.
- `weight_terminal_lat_error`: Preferable to set a higher value than normal lateral weight `weight_lat_error` for stability.
- `weight_terminal_heading_error`: Preferable to set a higher value than normal heading weight `weight_heading_error` for stability.

# Related issues
<!-- Required -->
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1057
