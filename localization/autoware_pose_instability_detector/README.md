# autoware_pose_instability_detector

The `pose_instability_detector` is a node designed to monitor the stability of `/localization/kinematic_state`, which is an output topic of the Extended Kalman Filter (EKF).

This node triggers periodic timer callbacks to compare two poses:

- The pose calculated by dead reckoning starting from the pose of `/localization/kinematic_state` obtained `timer_period` seconds ago.
- The latest pose from `/localization/kinematic_state`.

The results of this comparison are then output to the `/diagnostics` topic.

![overview](./media/pose_instability_detector_overview.png)

![rqt_runtime_monitor](./media/rqt_runtime_monitor.png)

If this node outputs WARN messages to `/diagnostics`, it means that the EKF output is significantly different from the integrated twist values.
In other words, WARN outputs indicate that the vehicle has moved to a place outside the expected range based on the twist values.
This discrepancy suggests that there may be an issue with either the estimated pose or the input twist.

The following diagram provides an overview of how the procedure looks like:

![procedure](./media/pose_instabilty_detector_procedure.svg)

## Dead reckoning algorithm

Dead reckoning is a method of estimating the position of a vehicle based on its previous position and velocity.
The procedure for dead reckoning is as follows:

1. Capture the necessary twist values from the `/input/twist` topic.
2. Integrate the twist values to calculate the pose transition.
3. Apply the pose transition to the previous pose to obtain the current pose.

### Collecting twist values

The `pose_instability_detector` node collects the twist values from the `~/input/twist` topic to perform dead reckoning.
Ideally, `pose_instability_detector` needs the twist values between the previous pose and the current pose.
Therefore, `pose_instability_detector` snips the twist buffer and apply interpolations and extrapolations to obtain the twist values at the desired time.

![how_to_snip_necessary_twist](./media/how_to_snip_twist.png)

### Linear transition and angular transition

After the twist values are collected, the node calculates the linear transition and angular transition based on the twist values and add them to the previous pose.

## Threshold definition

The `pose_instability_detector` node compares the pose calculated by dead reckoning with the latest pose from the EKF output.
These two pose are ideally the same, but in reality, they are not due to the error in the twist values the pose observation.
If these two poses are significantly different so that the absolute value exceeds the threshold, the node outputs a WARN message to the `/diagnostics` topic.
There are six thresholds (x, y, z, roll, pitch, and yaw) to determine whether the poses are significantly different, and these thresholds are determined by the following subsections.

### `diff_position_x`

This threshold examines the difference in the longitudinal axis between the two poses, and check whether the vehicle goes beyond the expected error.
This threshold is a sum of "maximum longitudinal error due to velocity scale factor error" and "pose estimation error tolerance".

$$
\tau_x = v_{\rm max}\frac{\beta_v}{100} \Delta t + \epsilon_x\\
$$

| Symbol        | Description                                                                      | Unit  |
| ------------- | -------------------------------------------------------------------------------- | ----- |
| $\tau_x$      | Threshold for the difference in the longitudinal axis                            | $m$   |
| $v_{\rm max}$ | Maximum velocity                                                                 | $m/s$ |
| $\beta_v$     | Scale factor tolerance for the maximum velocity                                  | $\%$  |
| $\Delta t$    | Time interval                                                                    | $s$   |
| $\epsilon_x$  | Pose estimator (e. g. ndt_scan_matcher) error tolerance in the longitudinal axis | $m$   |

### `diff_position_y` and `diff_position_z`

These thresholds examine the difference in the lateral and vertical axes between the two poses, and check whether the vehicle goes beyond the expected error.
The `pose_instability_detector` calculates the possible range where the vehicle goes, and get the maximum difference between the nominal dead reckoning pose and the maximum limit pose.

![lateral_threshold_calculation](./media/lateral_threshold_calculation.png)

Addition to this, the `pose_instability_detector` node considers the pose estimation error tolerance to determine the threshold.

$$
\tau_y = l + \epsilon_y
$$

| Symbol       | Description                                                                                     | Unit |
| ------------ | ----------------------------------------------------------------------------------------------- | ---- |
| $\tau_y$     | Threshold for the difference in the lateral axis                                                | $m$  |
| $l$          | Maximum lateral distance described in the image above (See the appendix how this is calculated) | $m$  |
| $\epsilon_y$ | Pose estimator (e. g. ndt_scan_matcher) error tolerance in the lateral axis                     | $m$  |

Note that `pose_instability_detector` sets the threshold for the vertical axis as the same as the lateral axis. Only the pose estimator error tolerance is different.

### `diff_angle_x`, `diff_angle_y`, and `diff_angle_z`

These thresholds examine the difference in the roll, pitch, and yaw angles between the two poses.
This threshold is a sum of "maximum angular error due to velocity scale factor error and bias error" and "pose estimation error tolerance".

$$
\tau_\phi = \tau_\theta = \tau_\psi = \left(\omega_{\rm max}\frac{\beta_\omega}{100} + b \right) \Delta t + \epsilon_\psi
$$

| Symbol             | Description                                                              | Unit          |
| ------------------ | ------------------------------------------------------------------------ | ------------- |
| $\tau_\phi$        | Threshold for the difference in the roll angle                           | ${\rm rad}$   |
| $\tau_\theta$      | Threshold for the difference in the pitch angle                          | ${\rm rad}$   |
| $\tau_\psi$        | Threshold for the difference in the yaw angle                            | ${\rm rad}$   |
| $\omega_{\rm max}$ | Maximum angular velocity                                                 | ${\rm rad}/s$ |
| $\beta_\omega$     | Scale factor tolerance for the maximum angular velocity                  | $\%$          |
| $b$                | Bias tolerance of the angular velocity                                   | ${\rm rad}/s$ |
| $\Delta t$         | Time interval                                                            | $s$           |
| $\epsilon_\psi$    | Pose estimator (e. g. ndt_scan_matcher) error tolerance in the yaw angle | ${\rm rad}$   |

## Parameters

{{ json_to_markdown("localization/autoware_pose_instability_detector/schema/pose_instability_detector.schema.json") }}

## Input

| Name               | Type                                           | Description           |
| ------------------ | ---------------------------------------------- | --------------------- |
| `~/input/odometry` | nav_msgs::msg::Odometry                        | Pose estimated by EKF |
| `~/input/twist`    | geometry_msgs::msg::TwistWithCovarianceStamped | Twist                 |

## Output

| Name                | Type                                  | Description |
| ------------------- | ------------------------------------- | ----------- |
| `~/debug/diff_pose` | geometry_msgs::msg::PoseStamped       | diff_pose   |
| `/diagnostics`      | diagnostic_msgs::msg::DiagnosticArray | Diagnostics |

## Appendix

On calculating the maximum lateral distance $l$, the `pose_instability_detector` node will estimate the following poses.

| Pose                            | heading velocity $v$                             | angular velocity $\omega$                                      |
| ------------------------------- | ------------------------------------------------ | -------------------------------------------------------------- |
| Nominal dead reckoning pose     | $v_{\rm max}$                                    | $\omega_{\rm max}$                                             |
| Dead reckoning pose of corner A | $\left(1+\frac{\beta_v}{100}\right) v_{\rm max}$ | $\left(1+\frac{\beta_\omega}{100}\right) \omega_{\rm max} + b$ |
| Dead reckoning pose of corner B | $\left(1-\frac{\beta_v}{100}\right) v_{\rm max}$ | $\left(1+\frac{\beta_\omega}{100}\right) \omega_{\rm max} + b$ |
| Dead reckoning pose of corner C | $\left(1-\frac{\beta_v}{100}\right) v_{\rm max}$ | $\left(1-\frac{\beta_\omega}{100}\right) \omega_{\rm max} - b$ |
| Dead reckoning pose of corner D | $\left(1+\frac{\beta_v}{100}\right) v_{\rm max}$ | $\left(1-\frac{\beta_\omega}{100}\right) \omega_{\rm max} - b$ |

Given a heading velocity $v$ and $\omega$, the 2D theoretical variation seen from the previous pose is calculated as follows:

$$
\begin{align*}
\left[
    \begin{matrix}
    \Delta x\\
    \Delta y
    \end{matrix}
\right]
&=
\left[
    \begin{matrix}
    \int_{0}^{\Delta t} v \cos(\omega t) dt\\
    \int_{0}^{\Delta t} v \sin(\omega t) dt
    \end{matrix}
\right]
\\
&=
\left[
    \begin{matrix}
    \frac{v}{\omega} \sin(\omega \Delta t)\\
    \frac{v}{\omega} \left(1 - \cos(\omega \Delta t)\right)
    \end{matrix}
\right]
\end{align*}
$$

We calculate this variation for each corner and get the maximum value of the lateral distance $l$ by comparing the distance between the nominal dead reckoning pose and the corner poses.
