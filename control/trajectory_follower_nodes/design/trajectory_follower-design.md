# Trajectory Follower Nodes

## Purpose

Generate control commands to follow a given Trajectory.

## Design

This is a node of the functionalities implemented in [trajectory_follower](../../trajectory_follower/design/trajectory_follower-design.md#trajectory-follower) package. It has instances of those functionalities, gives them input data to perform calculations, and publishes control commands.

![Controller](media/Controller.drawio.svg)

The process flow is as follows.

1. Set input data to the lateral controller
2. Compute lateral commands.
3. Sync the result of the lateral control to the longitudinal controller.
   - steer angle convergence
4. Set input data to the longitudinal controller
5. Compute longitudinal commands.
6. Sync the result of the longitudinal control to the longitudinal controller.
   - velocity convergence(currently not used)

Giving the longitudinal controller information about steer convergence allows it to control steer when stopped if following parameters are `true`

- lateral controller
  - `keep_steer_control_until_converged`
- longitudinal controller
  - `enable_keep_stopped_until_steer_convergence`

### Inputs / Outputs / API

#### Inputs

- `autoware_auto_planning_msgs/Trajectory` : reference trajectory to follow.
- `nav_msgs/Odometry`: current odometry
- `autoware_auto_vehicle_msgs/SteeringReport` current steering

#### Outputs

- `autoware_auto_control_msgs/AckermannControlCommand`: message containing both lateral and longitudinal commands.

#### Parameter

- `ctrl_period`: control commands publishing period
- `timeout_thr_sec`: duration in second after which input messages are discarded.
  - Each time the node receives lateral and longitudinal commands from each controller, it publishes an `AckermannControlCommand` if the following two conditions are met.
    1. Both commands have been received.
    2. The last received commands are not older than defined by `timeout_thr_sec`.
- `lateral_controller_mode`: `mpc_follower` or `pure_pursuit`
  - (currently there is only `PID` for longitudinal controller)

## Debugging

Debug information are published by the lateral and longitudinal controller using `tier4_debug_msgs/Float32MultiArrayStamped` messages.

A configuration file for [PlotJuggler](https://github.com/facontidavide/PlotJuggler) is provided in the `config` folder which, when loaded, allow to automatically subscribe and visualize information useful for debugging.

In addition, the predicted MPC trajectory is published on topic `output/lateral/predicted_trajectory` and can be visualized in Rviz.
