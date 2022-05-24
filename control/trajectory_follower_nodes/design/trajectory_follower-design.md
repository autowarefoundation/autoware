# Trajectory Follower Nodes

## Purpose

Generate control commands to follow a given Trajectory.

## Design

This functionality is decomposed into three nodes.

- [lateral-controller-design](lateral_controller-design.md) : generates lateral control messages.
- [longitudinal-controller-design](longitudinal_controller-design.md) : generates longitudinal control messages.
- [latlon-muxer-design](latlon_muxer-design.md) : combines the lateral and longitudinal control commands
  into a single control command.

Core functionalities are implemented in the [trajectory_follower](../../trajectory_follower/design/trajectory_follower-design.md#trajectory-follower) package.

@image html images/trajectory_follower-diagram.png "Overview of the Trajectory Follower package"

## Debugging

Debug information are published by the lateral and longitudinal controller using `autoware_auto_msgs/Float32MultiArrayDiagnostic` messages.

A configuration file for [PlotJuggler](https://github.com/facontidavide/PlotJuggler) is provided in the `config` folder which, when loaded, allow to automatically subscribe and visualize information useful for debugging.

In addition, the predicted MPC trajectory is published on topic `output/lateral/predicted_trajectory` and can be visualized in Rviz.
