# simple_planning_simulator

## Purpose / Use cases

This node simulates the vehicle motion for a vehicle command in 2D using a simple vehicle model.

## Design

The purpose of this simulator is for the integration test of planning and control modules. This does not simulate sensing or perception, but is implemented in pure c++ only and works without GPU.

## Assumptions / Known limits

- It simulates only in 2D motion.
- It does not perform physical operations such as collision and sensing, but only calculates the integral results of vehicle dynamics.

## Inputs / Outputs / API

### input

- input/initialpose [`geometry_msgs/msg/PoseWithCovarianceStamped`] : for initial pose
- input/ackermann_control_command [`autoware_auto_msgs/msg/AckermannControlCommand`] : target command to drive a vehicle
- input/manual_ackermann_control_command [`autoware_auto_msgs/msg/AckermannControlCommand`] : manual target command to drive a vehicle (used when control_mode_request = Manual)
- input/gear_command [`autoware_auto_vehicle_msgs/msg/GearCommand`] : target gear command.
- input/manual_gear_command [`autoware_auto_vehicle_msgs/msg/GearCommand`] : target gear command (used when control_mode_request = Manual)
- input/turn_indicators_command [`autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand`] : target turn indicator command
- input/hazard_lights_command [`autoware_auto_vehicle_msgs/msg/HazardLightsCommand`] : target hazard lights command
- input/control_mode_request [`tier4_vehicle_msgs::srv::ControlModeRequest`] : mode change for Auto/Manual driving

### output

- /tf [`tf2_msgs/msg/TFMessage`] : simulated vehicle pose (base_link)
- /output/odometry [`nav_msgs/msg/Odometry`] : simulated vehicle pose and twist
- /output/steering [`autoware_auto_vehicle_msgs/msg/SteeringReport`] : simulated steering angle
- /output/control_mode_report [`autoware_auto_vehicle_msgs/msg/ControlModeReport`] : current control mode (Auto/Manual)
- /output/gear_report [`autoware_auto_vehicle_msgs/msg/ControlModeReport`] : simulated gear
- /output/turn_indicators_report [`autoware_auto_vehicle_msgs/msg/ControlModeReport`] : simulated turn indicator status
- /output/hazard_lights_report [`autoware_auto_vehicle_msgs/msg/ControlModeReport`] : simulated hazard lights status

## Inner-workings / Algorithms

### Common Parameters

| Name                   | Type   | Description                                                                                                                                | Default value        |
| :--------------------- | :----- | :----------------------------------------------------------------------------------------------------------------------------------------- | :------------------- |
| simulated_frame_id     | string | set to the child_frame_id in output tf                                                                                                     | "base_link"          |
| origin_frame_id        | string | set to the frame_id in output tf                                                                                                           | "odom"               |
| initialize_source      | string | If "ORIGIN", the initial pose is set at (0,0,0). If "INITIAL_POSE_TOPIC", node will wait until the `input/initialpose` topic is published. | "INITIAL_POSE_TOPIC" |
| add_measurement_noise  | bool   | If true, the Gaussian noise is added to the simulated results.                                                                             | true                 |
| pos_noise_stddev       | double | Standard deviation for position noise                                                                                                      | 0.01                 |
| rpy_noise_stddev       | double | Standard deviation for Euler angle noise                                                                                                   | 0.0001               |
| vel_noise_stddev       | double | Standard deviation for longitudinal velocity noise                                                                                         | 0.0                  |
| angvel_noise_stddev    | double | Standard deviation for angular velocity noise                                                                                              | 0.0                  |
| steer_noise_stddev     | double | Standard deviation for steering angle noise                                                                                                | 0.0001               |
| measurement_steer_bias | double | Measurement bias for steering angle                                                                                                        | 0.0                  |

### Vehicle Model Parameters

#### vehicle_model_type options

- `IDEAL_STEER_VEL`
- `IDEAL_STEER_ACC`
- `IDEAL_STEER_ACC_GEARED`
- `DELAY_STEER_VEL`
- `DELAY_STEER_ACC`
- `DELAY_STEER_ACC_GEARED`
- `DELAY_STEER_MAP_ACC_GEARED`: applies 1D dynamics and time delay to the steering and acceleration commands. The simulated acceleration is determined by a value converted through the provided acceleration map. This model is valuable for an accurate simulation with acceleration deviations in a real vehicle.

The `IDEAL` model moves ideally as commanded, while the `DELAY` model moves based on a 1st-order with time delay model. The `STEER` means the model receives the steer command. The `VEL` means the model receives the target velocity command, while the `ACC` model receives the target acceleration command. The `GEARED` suffix means that the motion will consider the gear command: the vehicle moves only one direction following the gear.

The table below shows which models correspond to what parameters. The model names are written in abbreviated form (e.g. IDEAL_STEER_VEL = I_ST_V).

| Name                       | Type   | Description                                                                                                 | I_ST_V | I_ST_A | I_ST_A_G | D_ST_V | D_ST_A | D_ST_A_G | D_ST_M_ACC_G | Default value | unit    |
| :------------------------- | :----- | :---------------------------------------------------------------------------------------------------------- | :----- | :----- | :------- | :----- | :----- | :------- | :----------- | :------------ | :------ |
| acc_time_delay             | double | dead time for the acceleration input                                                                        | x      | x      | x        | x      | o      | o        | o            | 0.1           | [s]     |
| steer_time_delay           | double | dead time for the steering input                                                                            | x      | x      | x        | o      | o      | o        | o            | 0.24          | [s]     |
| vel_time_delay             | double | dead time for the velocity input                                                                            | x      | x      | x        | o      | x      | x        | x            | 0.25          | [s]     |
| acc_time_constant          | double | time constant of the 1st-order acceleration dynamics                                                        | x      | x      | x        | x      | o      | o        | o            | 0.1           | [s]     |
| steer_time_constant        | double | time constant of the 1st-order steering dynamics                                                            | x      | x      | x        | o      | o      | o        | o            | 0.27          | [s]     |
| steer_dead_band            | double | dead band for steering angle                                                                                | x      | x      | x        | o      | o      | o        | x            | 0.0           | [rad]   |
| vel_time_constant          | double | time constant of the 1st-order velocity dynamics                                                            | x      | x      | x        | o      | x      | x        | x            | 0.5           | [s]     |
| vel_lim                    | double | limit of velocity                                                                                           | x      | x      | x        | o      | o      | o        | o            | 50.0          | [m/s]   |
| vel_rate_lim               | double | limit of acceleration                                                                                       | x      | x      | x        | o      | o      | o        | o            | 7.0           | [m/ss]  |
| steer_lim                  | double | limit of steering angle                                                                                     | x      | x      | x        | o      | o      | o        | o            | 1.0           | [rad]   |
| steer_rate_lim             | double | limit of steering angle change rate                                                                         | x      | x      | x        | o      | o      | o        | o            | 5.0           | [rad/s] |
| debug_acc_scaling_factor   | double | scaling factor for accel command                                                                            | x      | x      | x        | x      | o      | o        | x            | 1.0           | [-]     |
| debug_steer_scaling_factor | double | scaling factor for steer command                                                                            | x      | x      | x        | x      | o      | o        | x            | 1.0           | [-]     |
| acceleration_map_path      | string | path to csv file for acceleration map which converts velocity and ideal acceleration to actual acceleration | x      | x      | x        | x      | x      | x        | o            | -             | [-]     |

The `acceleration_map` is used only for `DELAY_STEER_MAP_ACC_GEARED` and it shows the acceleration command on the vertical axis and the current velocity on the horizontal axis, with each cell representing the converted acceleration command that is actually used in the simulator's motion calculation. Values in between are linearly interpolated.

Example of `acceleration_map.csv`

```csv
default,  0.00,  1.39,  2.78,  4.17,  5.56,  6.94,  8.33,  9.72, 11.11, 12.50, 13.89, 15.28, 16.67
-4.0,    -4.40, -4.36, -4.38, -4.12, -4.20, -3.94, -3.98, -3.80, -3.77, -3.76, -3.59, -3.50, -3.40
-3.5,    -4.00, -3.91, -3.85, -3.64, -3.68, -3.55, -3.42, -3.24, -3.25, -3.00, -3.04, -2.93, -2.80
-3.0,    -3.40, -3.37, -3.33, -3.00, -3.00, -2.90, -2.88, -2.65, -2.43, -2.44, -2.43, -2.39, -2.30
-2.5,    -2.80, -2.72, -2.72, -2.62, -2.41, -2.43, -2.26, -2.18, -2.11, -2.03, -1.96, -1.91, -1.85
-2.0,    -2.30, -2.24, -2.12, -2.02, -1.92, -1.81, -1.67, -1.58, -1.51, -1.49, -1.40, -1.35, -1.30
-1.5,    -1.70, -1.61, -1.47, -1.46, -1.40, -1.37, -1.29, -1.24, -1.10, -0.99, -0.83, -0.80, -0.78
-1.0,    -1.30, -1.28, -1.10, -1.09, -1.04, -1.02, -0.98, -0.89, -0.82, -0.61, -0.52, -0.54, -0.56
-0.8,    -0.96, -0.90, -0.82, -0.74, -0.70, -0.65, -0.63, -0.59, -0.55, -0.44, -0.39, -0.39, -0.35
-0.6,    -0.77, -0.71, -0.67, -0.65, -0.58, -0.52, -0.51, -0.50, -0.40, -0.33, -0.30, -0.31, -0.30
-0.4,    -0.45, -0.40, -0.45, -0.44, -0.38, -0.35, -0.31, -0.30, -0.26, -0.30, -0.29, -0.31, -0.25
-0.2,    -0.24, -0.24, -0.25, -0.22, -0.23, -0.25, -0.27, -0.29, -0.24, -0.22, -0.17, -0.18, -0.12
 0.0,     0.00,  0.00, -0.05, -0.05, -0.05, -0.05, -0.08, -0.08, -0.08, -0.08, -0.10, -0.10, -0.10
 0.2,     0.16,  0.12,  0.02,  0.02,  0.00,  0.00, -0.05, -0.05, -0.05, -0.05, -0.08, -0.08, -0.08
 0.4,     0.38,  0.30,  0.22,  0.25,  0.24,  0.23,  0.20,  0.16,  0.16,  0.14,  0.10,  0.05,  0.05
 0.6,     0.52,  0.52,  0.51,  0.49,  0.43,  0.40,  0.35,  0.33,  0.33,  0.33,  0.32,  0.34,  0.34
 0.8,     0.82,  0.81,  0.78,  0.68,  0.63,  0.56,  0.53,  0.48,  0.43,  0.41,  0.37,  0.38,  0.40
 1.0,     1.00,  1.08,  1.01,  0.88,  0.76,  0.69,  0.66,  0.58,  0.54,  0.49,  0.45,  0.40,  0.40
 1.5,     1.52,  1.50,  1.38,  1.26,  1.14,  1.03,  0.91,  0.82,  0.67,  0.61,  0.51,  0.41,  0.41
 2.0,     1.80,  1.80,  1.64,  1.43,  1.25,  1.11,  0.96,  0.81,  0.70,  0.59,  0.51,  0.42,  0.42
```

![acceleration_map](./media/acceleration_map.svg)

<!-- deadzone_delta_steer | double | dead zone for the steering dynamics                  | x      | x      | x        | o      | o      | 0.0      | [rad]         |         | -->

_Note_: The steering/velocity/acceleration dynamics is modeled by a first order system with a deadtime in a _delay_ model. The definition of the _time constant_ is the time it takes for the step response to rise up to 63% of its final value. The _deadtime_ is a delay in the response to a control input.

### Default TF configuration

Since the vehicle outputs `odom`->`base_link` tf, this simulator outputs the tf with the same frame_id configuration.
In the simple_planning_simulator.launch.py, the node that outputs the `map`->`odom` tf, that usually estimated by the localization module (e.g. NDT), will be launched as well. Since the tf output by this simulator module is an ideal value, `odom`->`map` will always be 0.

### (Caveat) Pitch calculation

Ego vehicle pitch angle is calculated in the following manner.

![pitch calculation](./media/pitch-calculation.drawio.svg)

NOTE: driving against the line direction (as depicted in image's bottom row) is not supported and only shown for illustration purposes.

## Error detection and handling

The only validation on inputs being done is testing for a valid vehicle model type.

## Security considerations

<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

## References / External links

This is originally developed in the Autoware.AI. See the link below.

<https://github.com/Autoware-AI/simulation/tree/master/wf_simulator>

## Future extensions / Unimplemented parts

- Improving the accuracy of vehicle models (e.g., adding steering dead zones and slip behavior)
- Cooperation with modules that output pseudo pointcloud or pseudo perception results
