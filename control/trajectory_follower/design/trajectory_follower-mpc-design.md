MPC (Trajectory Follower) {#trajectory_follower-mpc-design}
===========

This is the design document for the MPC implemented in the `trajectory_follower` package.

# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
Model Predictive Control (MPC) is used by the `trajectory_follower`
to calculate the lateral commands corresponding to a steering angle and a steering rate.

This implementation differs from the one in the `mpc_controller` package in several aspects.
- This is a linear MPC that only computes the steering command whereas
the `mpc_controller` uses a non-linear MPC which calculates coupled steering and velocity commands.
- The optimization problem solved by the linear MPC is simpler, making it less likely to fail.
- Tuning of the linear MPC is easier.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
MPC uses predictions of the vehicle's motion to optimize the immediate control command.

Different vehicle models are implemented:
- `kinematics` : bicycle kinematics model with steering 1st-order delay.
- `kinematics_no_delay` : bicycle kinematics model without steering delay.
- `dynamics` : bicycle dynamics model considering slip angle.

The `kinematics` model is being used by default. Please see the reference [1] for more details.


For the optimization, a Quadratric Programming (QP) solver is used
with two options are currently implemented:
- `unconstraint` : use least square method to solve unconstraint QP with eigen.
- `unconstraint_fast` : similar to unconstraint. This is faster, but lower accuracy for optimization.

## Filtering

Filtering is required for good noise reduction.
A [Butterworth filter](https://en.wikipedia.org/wiki/Butterworth_filter) is used for the yaw and lateral errors used as input of the MPC as well as for
the output steering angle.
Other filtering methods can be considered as long as the noise reduction performances are good
enough.
The moving average filter for example is not suited and can yield worse results than without any
filtering.

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
The `MPC` class (defined in `mpc.hpp`) provides the interface with the MPC algorithm.
Once a vehicle model, a QP solver, and the reference trajectory to follow have been set
(using `setVehicleModel()`, `setQPSolver()`, `setReferenceTrajectory()`), a lateral control command
can be calculated by providing the current steer, velocity, and pose to function `calculateMPC()`.

# References / External links
<!-- Optional -->
- [1] Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking",
Robotics Institute, Carnegie Mellon University, February 2009.

# Related issues
<!-- Required -->
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1057
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1058
