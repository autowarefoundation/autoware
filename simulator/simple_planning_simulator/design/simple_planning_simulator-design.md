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

| Name                  | Type   | Description                                                                                                                                | Default value        |
| :-------------------- | :----- | :----------------------------------------------------------------------------------------------------------------------------------------- | :------------------- |
| simulated_frame_id    | string | set to the child_frame_id in output tf                                                                                                     | "base_link"          |
| origin_frame_id       | string | set to the frame_id in output tf                                                                                                           | "odom"               |
| initialize_source     | string | If "ORIGIN", the initial pose is set at (0,0,0). If "INITIAL_POSE_TOPIC", node will wait until the `input/initialpose` topic is published. | "INITIAL_POSE_TOPIC" |
| add_measurement_noise | bool   | If true, the Gaussian noise is added to the simulated results.                                                                             | true                 |
| pos_noise_stddev      | double | Standard deviation for position noise                                                                                                      | 0.01                 |
| rpy_noise_stddev      | double | Standard deviation for Euler angle noise                                                                                                   | 0.0001               |
| vel_noise_stddev      | double | Standard deviation for longitudinal velocity noise                                                                                         | 0.0                  |
| angvel_noise_stddev   | double | Standard deviation for angular velocity noise                                                                                              | 0.0                  |
| steer_noise_stddev    | double | Standard deviation for steering angle noise                                                                                                | 0.0001               |

### Vehicle Model Parameters

#### vehicle_model_type options

- `IDEAL_STEER_VEL`
- `IDEAL_STEER_ACC`
- `IDEAL_STEER_ACC_GEARED`
- `DELAY_STEER_VEL`
- `DELAY_STEER_ACC`
- `DELAY_STEER_ACC_GEARED`

The `IDEAL` model moves ideally as commanded, while the `DELAY` model moves based on a 1st-order with time delay model. The `STEER` means the model receives the steer command. The `VEL` means the model receives the target velocity command, while the `ACC` model receives the target acceleration command. The `GEARED` suffix means that the motion will consider the gear command: the vehicle moves only one direction following the gear.

The table below shows which models correspond to what parameters. The model names are written in abbreviated form (e.g. IDEAL_STEER_VEL = I_ST_V).

| Name                | Type   | Description                                          | I_ST_V | I_ST_A | I_ST_A_G | D_ST_V | D_ST_A | D_ST_A_G | Default value | unit    |
| :------------------ | :----- | :--------------------------------------------------- | :----- | :----- | :------- | :----- | :----- | :------- | :------------ | :------ |
| acc_time_delay      | double | dead time for the acceleration input                 | x      | x      | x        | x      | o      | o        | 0.1           | [s]     |
| steer_time_delay    | double | dead time for the steering input                     | x      | x      | x        | o      | o      | o        | 0.24          | [s]     |
| vel_time_delay      | double | dead time for the velocity input                     | x      | x      | x        | o      | x      | x        | 0.25          | [s]     |
| acc_time_constant   | double | time constant of the 1st-order acceleration dynamics | x      | x      | x        | x      | o      | o        | 0.1           | [s]     |
| steer_time_constant | double | time constant of the 1st-order steering dynamics     | x      | x      | x        | o      | o      | o        | 0.27          | [s]     |
| vel_time_constant   | double | time constant of the 1st-order velocity dynamics     | x      | x      | x        | o      | x      | x        | 0.5           | [s]     |
| vel_lim             | double | limit of velocity                                    | x      | x      | x        | o      | o      | o        | 50.0          | [m/s]   |
| vel_rate_lim        | double | limit of acceleration                                | x      | x      | x        | o      | o      | o        | 7.0           | [m/ss]  |
| steer_lim           | double | limit of steering angle                              | x      | x      | x        | o      | o      | o        | 1.0           | [rad]   |
| steer_rate_lim      | double | limit of steering angle change rate                  | x      | x      | x        | o      | o      | o        | 5.0           | [rad/s] |

<!-- deadzone_delta_steer | double | dead zone for the steering dynamics                  | x      | x      | x        | o      | o      | 0.0      | [rad]         |         | -->

_Note_: The steering/velocity/acceleration dynamics is modeled by a first order system with a deadtime in a _delay_ model. The definition of the _time constant_ is the time it takes for the step response to rise up to 63% of its final value. The _deadtime_ is a delay in the response to a control input.

### Default TF configuration

Since the vehicle outputs `odom`->`base_link` tf, this simulator outputs the tf with the same frame_id configuration.
In the simple_planning_simulator.launch.py, the node that outputs the `map`->`odom` tf, that usually estimated by the localization module (e.g. NDT), will be launched as well. Since the tf output by this simulator module is an ideal value, `odom`->`map` will always be 0.

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
