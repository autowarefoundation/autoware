# Trajectory Follower

This is the design document for the `trajectory_follower` package.

## Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This package provides the library code used by the nodes of the `trajectory_follower_nodes` package.
Mainly, it implements two algorithms:

- Model-Predictive Control (MPC) for the computation of lateral steering commands.
  - [trajectory_follower-mpc-design](mpc_lateral_controller-design.md)
- PID control for the computation of velocity and acceleration commands.
  - [trajectory_follower-pid-design](pid_longitudinal_controller-design.md)

## Design

![Controller](../../trajectory_follower_nodes/design/media/Controller.drawio.svg)

There are lateral and longitudinal base interface classes and each algorithm inherits from this class to implement.
The interface class has the following base functions.

- `setInputData()`: Input the data subscribed in [Trajectory Follower Nodes](../../trajectory_follower_nodes/design/trajectory_follower-design.md). This must be implemented with the inherited algorithm and the used data must be selected.
- `run()`: Compute control commands and return to [Trajectory Follower Nodes](../../trajectory_follower_nodes/design/trajectory_follower-design.md). This must be implemented by inherited algorithms.
- `syncData()`: Input the result of running the other controller.
  - steer angle convergence
    - allow keeping stopped until steer is converged.
  - velocity convergence(currently not used)

See [the Design of Trajectory Follower Nodes](../../trajectory_follower_nodes/design/trajectory_follower-design.md#Design) for how these functions work in the node.

## Separated lateral (steering) and longitudinal (velocity) controls

This longitudinal controller assumes that the roles of lateral and longitudinal control are separated as follows.

- Lateral control computes a target steering to keep the vehicle on the trajectory, assuming perfect velocity tracking.
- Longitudinal control computes a target velocity/acceleration to keep the vehicle velocity on the trajectory speed, assuming perfect trajectory tracking.

Ideally, dealing with the lateral and longitudinal control as a single mixed problem can achieve high performance. In contrast, there are two reasons to provide velocity controller as a stand-alone function, described below.

### Complex requirements for longitudinal motion

The longitudinal vehicle behavior that humans expect is difficult to express in a single logic. For example, the expected behavior just before stopping differs depending on whether the ego-position is ahead/behind of the stop line, or whether the current speed is higher/lower than the target speed to achieve a human-like movement.

In addition, some vehicles have difficulty measuring the ego-speed at extremely low speeds. In such cases, a configuration that can improve the functionality of the longitudinal control without affecting the lateral control is important.

There are many characteristics and needs that are unique to longitudinal control. Designing them separately from the lateral control keeps the modules less coupled and improves maintainability.

### Nonlinear coupling of lateral and longitudinal motion

The lat-lon mixed control problem is very complex and uses nonlinear optimization to achieve high performance. Since it is difficult to guarantee the convergence of the nonlinear optimization, a simple control logic is also necessary for development.

Also, the benefits of simultaneous longitudinal and lateral control are small if the vehicle doesn't move at high speed.

## Related issues

<!-- Required -->
