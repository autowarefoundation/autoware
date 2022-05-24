# Trajectory Follower

This is the design document for the `trajectory_follower` package.

## Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This package provides the library code used by the nodes of the `trajectory_follower_nodes` package.
Mainly, it implements two algorithms:

- Model-Predictive Control (MPC) for the computation of lateral steering commands.
  - [trajectory_follower-mpc-design](trajectory_follower-mpc-design.md)
- PID control for the computation of velocity and acceleration commands.
  - [trajectory_follower-pid-design](trajectory_follower-pid-design.md)

## Related issues

<!-- Required -->

- <https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1057>
- <https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1058>
