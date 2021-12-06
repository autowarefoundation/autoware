PID (Trajectory Follower) {#trajectory_follower-pid-design}
===========

This is the design document for the PID implemented in the `trajectory_follower` package.

# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
PID control is used by the `trajectory_follower`
to calculate longitudinal commands corresponding to a velocity and an acceleration.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
This PID control calculates the target acceleration from the deviation between the current ego-velocity and the target velocity.

This PID logic has a maximum value for the output of each term. This is to prevent the following:

- Large integral terms may cause unintended behavior by users.
- Unintended noise may cause the output of the derivative term to be very large.

Also, the integral term is not accumulated when the vehicle is stopped. This is to prevent unintended accumulation of the integral term in cases such as "Autoware assumes that the vehicle is engaged, but an external system has locked the vehicle to start.
On the other hand, if the vehicle gets stuck in a depression in the road surface when starting, the vehicle will not start forever, which is currently being addressed by developers.

At present, PID control is implemented from the viewpoint of trade-off between development/maintenance cost and performance.
This may be replaced by a higher performance controller (adaptive control or robust control) in future development.

@image html images/trajectory_follower-pid-diagram.svg "PID controller diagram"

## States

This module has four state transitions as shown below in order to handle special processing in a specific situation.

- **DRIVE**
  - Executes target velocity tracking by PID control.
  - It also applies the delay compensation and slope compensation.
- **STOPPING**
  - Controls the motion just before stopping.
  - Special sequence is performed to achieve accurate and smooth stopping.
- **STOPPED**
  - Performs operations in the stopped state (e.g. brake hold)
- **EMERGENCY**.
  - Enters an emergency state when certain conditions are met (e.g., when the vehicle has crossed a certain distance of a stop line).
  - The recovery condition (whether or not to keep emergency state until the vehicle completely stops) or the deceleration in the emergency state are defined by parameters.

The state transition diagram is shown below.

@image html images/trajectory_follower-pid-states-diagram.svg "State Transitions"

## Time delay compensation

At high speeds, the delay of actuator systems such as gas pedals and brakes has a significant impact on driving accuracy.
Depending on the actuating principle of the vehicle,
the mechanism that physically controls the gas pedal and brake typically has a delay of about a hundred millisecond.

In this controller,
the predicted ego-velocity and the target velocity after the delay time are calculated and used for the feedback to address the time delay problem.

## Slope compensation

Based on the slope information, a compensation term is added to the target acceleration.

There are two sources of the slope information, which can be switched by a parameter.

- Pitch of the estimated ego-pose (default)
  - Calculates the current slope from the pitch angle of the estimated ego-pose
  - Pros: Easily available
  - Cons: Cannot extract accurate slope information due to the influence of vehicle vibration.
- Z coordinate on the trajectory
  - Calculates the road slope from the difference of z-coordinates between the front and rear wheel positions in the target trajectory
  - Pros: More accurate than pitch information, if the z-coordinates of the route are properly maintained
  - Pros: Can be used in combination with delay compensation (not yet implemented)
  - Cons: z-coordinates of high-precision map is needed.
  - Cons: Does not support free space planning (for now)

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
The `PIDController` class is straightforward to use.
First, gains and limits must be set (using `setGains()` and `setLimits()`) for the proportional (P), integral (I), and derivative (D) components.
Then, the velocity can be calculated by providing the current error and time step duration to the `calculate()` function.

# Related issues
<!-- Required -->
- https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1058
