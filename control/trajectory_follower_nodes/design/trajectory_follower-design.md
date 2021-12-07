Trajectory Follower Nodes {#trajectory_follower_nodes-package-design}
=============================================

# Purpose

Generate control commands to follow a given Trajectory.

# Design

This functionality is decomposed into three nodes.
- @subpage lateral-controller-design : generates lateral control messages.
- @subpage longitudinal-controller-design : generates longitudinal control messages.
- @subpage latlon-muxer-design : combines the lateral and longitudinal control commands
into a single control command.

Core functionalities are implemented in the @subpage trajectory_follower-package-design package.

@image html images/trajectory_follower-diagram.png "Overview of the Trajectory Follower package"

# Debugging

Debug information are published by the lateral and longitudinal controller using `autoware_auto_msgs/Float32MultiArrayDiagnostic` messages.

A configuration file for [PlotJuggler](https://github.com/facontidavide/PlotJuggler) is provided in the `config` folder which, when loaded, allow to automatically subscribe and visualize information useful for debugging.

In addition, the predicted MPC trajectory is published on topic `output/lateral/predicted_trajectory` and can be visualized in Rviz.