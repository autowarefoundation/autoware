Trajectory Follower {#trajectory_follower-package-design}
===========

This is the design document for the `trajectory_follower` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
This package provides the library code used by the nodes of the `trajectory_follower_nodes` package.
Mainly, it implements two algorithms:
- Model-Predictive Control (MPC) for the computation of lateral steering commands.
    - @subpage trajectory_follower-mpc-design
- PID control for the computation of velocity and acceleration commands.
    - @subpage trajectory_follower-pid-design

# Related issues
<!-- Required -->
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1057
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1058
