# Perception Evaluator

A node for evaluating the output of perception systems.

## Purpose

This module allows for the evaluation of how accurately perception results are generated without the need for annotations. It is capable of confirming performance and can evaluate results from a few seconds prior, enabling online execution.

## Inner-workings / Algorithms

The evaluated metrics are as follows:

- predicted_path_deviation
- predicted_path_deviation_variance
- lateral_deviation
- yaw_deviation
- yaw_rate
- total_objects_count
- average_objects_count
- interval_objects_count

### Predicted Path Deviation / Predicted Path Deviation Variance

Compare the predicted path of past objects with their actual traveled path to determine the deviation for **MOVING OBJECTS**. For each object, calculate the mean distance between the predicted path points and the corresponding points on the actual path, up to the specified time step. In other words, this calculates the Average Displacement Error (ADE). The target object to be evaluated is the object from $T_N$ seconds ago, where $T_N$ is the maximum value of the prediction time horizon $[T_1, T_2, ..., T_N]$.

> [!NOTE]
> The object from $T_N$ seconds ago is the target object for all metrics. This is to unify the time of the target object across metrics.

![path_deviation_each_object](./images/path_deviation_each_object.drawio.svg)

$$
\begin{align}
n_{points} = T / dt \\
ADE = \Sigma_{i=1}^{n_{points}} d_i / n_{points}　\\
Var = \Sigma_{i=1}^{n_{points}} (d_i - ADE)^2 / n_{points}
\end{align}
$$

- $n_{points}$ : Number of points in the predicted path
- $T$ : Time horizon for prediction evaluation.
- $dt$ : Time interval of the predicted path
- $d_i$ : Distance between the predicted path and the actual traveled path at path point $i$
- $ADE$ : Mean deviation of the predicted path for the target object.
- $Var$ : Variance of the predicted path deviation for the target object.

The final predicted path deviation metrics are calculated by averaging the mean deviation of the predicted path for all objects of the same class, and then calculating the mean, maximum, and minimum values of the mean deviation.

![path_deviation](./images/path_deviation.drawio.svg)

$$
\begin{align}
ADE_{mean} = \Sigma_{j=1}^{n_{objects}} ADE_j / n_{objects} \\
ADE_{max} = max(ADE_j) \\
ADE_{min} = min(ADE_j)
\end{align}
$$

$$
\begin{align}
Var_{mean} = \Sigma_{j=1}^{n_{objects}} Var_j / n_{objects} \\
Var_{max} = max(Var_j) \\
Var_{min} = min(Var_j)
\end{align}
$$

- $n_{objects}$ : Number of objects
- $ADE_{mean}$ : Mean deviation of the predicted path through all objects
- $ADE_{max}$ : Maximum deviation of the predicted path through all objects
- $ADE_{min}$ : Minimum deviation of the predicted path through all objects
- $Var_{mean}$ : Mean variance of the predicted path deviation through all objects
- $Var_{max}$ : Maximum variance of the predicted path deviation through all objects
- $Var_{min}$ : Minimum variance of the predicted path deviation through all objects

The actual metric name is determined by the object class and time horizon. For example, `predicted_path_deviation_variance_CAR_5.00`

### Lateral Deviation

Calculates lateral deviation between the smoothed traveled trajectory and the perceived position to evaluate the stability of lateral position recognition for **MOVING OBJECTS**. The smoothed traveled trajectory is calculated by applying a centered moving average filter whose window size is specified by the parameter `smoothing_window_size`. The lateral deviation is calculated by comparing the smoothed traveled trajectory with the perceived position of the past object whose timestamp is $T=T_n$ seconds ago. For stopped objects, the smoothed traveled trajectory is unstable, so this metric is not calculated.

![lateral_deviation](./images/lateral_deviation.drawio.svg)

### Yaw Deviation

Calculates the deviation between the recognized yaw angle of an past object and the yaw azimuth angle of the smoothed traveled trajectory for **MOVING OBJECTS**. The smoothed traveled trajectory is calculated by applying a centered moving average filter whose window size is specified by the parameter `smoothing_window_size`. The yaw deviation is calculated by comparing the yaw azimuth angle of smoothed traveled trajectory with the perceived orientation of the past object whose timestamp is $T=T_n$ seconds ago.
For stopped objects, the smoothed traveled trajectory is unstable, so this metric is not calculated.

![yaw_deviation](./images/yaw_deviation.drawio.svg)

### Yaw Rate

Calculates the yaw rate of an object based on the change in yaw angle from the previous time step. It is evaluated for **STATIONARY OBJECTS** and assesses the stability of yaw rate recognition. The yaw rate is calculated by comparing the yaw angle of the past object with the yaw angle of the object received in the previous cycle. Here, t2 is the timestamp that is $T_n$ seconds ago.

![yaw_rate](./images/yaw_rate.drawio.svg)

### Object Counts

Counts the number of detections for each object class within the specified detection range. These metrics are measured for the most recent object not past objects.

![detection_counts](./images/detection_counts.drawio.svg)

In the provided illustration, the range $R$ is determined by a combination of lists of radii (e.g., $r_1, r_2, \ldots$) and heights (e.g., $h_1, h_2, \ldots$).
For example,

- the number of CAR in range $R = (r_1, h_1)$ equals 1
- the number of CAR in range $R = (r_1, h_2)$ equals 2
- the number of CAR in range $R = (r_2, h_1)$ equals 3
- the number of CAR in range $R = (r_2, h_2)$ equals 4

#### Total Object Count

Counts the number of unique objects for each class within the specified detection range. The total object count is calculated as follows:

$$
\begin{align}
\text{Total Object Count (Class, Range)} = \left| \bigcup_{t=0}^{T_{\text{now}}} \{ \text{uuid} \mid \text{class}(t, \text{uuid}) = C \wedge \text{position}(t, \text{uuid}) \in R \} \right|
\end{align}
$$

where:

- $\bigcup$ represents the union across all frames from $t = 0$ to $T_{\text{now}}$, which ensures that each uuid is counted only once.
- $\text{class}(t, \text{uuid}) = C$ specifies that the object with uuid at time $t$ belongs to class $C$.
- $\text{position}(t, \text{uuid}) \in R$ indicates that the object with uuid at time $t$ is within the specified range $R$.
- $\left| \{ \ldots \} \right|$ denotes the cardinality of the set, which counts the number of unique uuids that meet the class and range criteria across all considered times.

#### Average Object Count

Counts the average number of objects for each class within the specified detection range. This metric measures how many objects were detected in one frame, without considering uuids. The average object count is calculated as follows:

$$
\begin{align}
\text{Average Object Count (Class, Range)} = \frac{1}{N} \sum_{t=0}^{T_{\text{now}}} \left| \{ \text{object} \mid \text{class}(t, \text{object}) = C \wedge \text{position}(t, \text{object}) \in R \} \right|
\end{align}
$$

where:

- $N$ represents the total number of frames within the time period time to $T\_{\text{now}}$ (it is precisely `detection_count_purge_seconds`)
- $text{object}$ denotes the number of objects that meet the class and range criteria at time $t$.

#### Interval Object Count

Counts the average number of objects for each class within the specified detection range over the last `objects_count_window_seconds`. This metric measures how many objects were detected in one frame, without considering uuids. The interval object count is calculated as follows:

$$
\begin{align}
\text{Interval Object Count (Class, Range)} = \frac{1}{W} \sum_{t=T_{\text{now}} - T_W}^{T_{\text{now}}} \left| \{ \text{object} \mid \text{class}(t, \text{object}) = C \wedge \text{position}(t, \text{object}) \in R \} \right|
\end{align}
$$

where:

- $W$ represents the total number of frames within the last `objects_count_window_seconds`.
- $T_W$ represents the time window `objects_count_window_seconds`

## Inputs / Outputs

| Name              | Type                                              | Description                                       |
| ----------------- | ------------------------------------------------- | ------------------------------------------------- |
| `~/input/objects` | `autoware_perception_msgs::msg::PredictedObjects` | The predicted objects to evaluate.                |
| `~/metrics`       | `diagnostic_msgs::msg::DiagnosticArray`           | Diagnostic information about perception accuracy. |
| `~/markers`       | `visualization_msgs::msg::MarkerArray`            | Visual markers for debugging and visualization.   |

## Parameters

| Name                                                   | Type         | Description                                                                                                                                     |
| ------------------------------------------------------ | ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `selected_metrics`                                     | List         | Metrics to be evaluated, such as lateral deviation, yaw deviation, and predicted path deviation.                                                |
| `smoothing_window_size`                                | Integer      | Determines the window size for smoothing path, should be an odd number.                                                                         |
| `prediction_time_horizons`                             | list[double] | Time horizons for prediction evaluation in seconds.                                                                                             |
| `stopped_velocity_threshold`                           | double       | threshold velocity to check if vehicle is stopped                                                                                               |
| `detection_radius_list`                                | list[double] | Detection radius for objects to be evaluated.(used for objects count only)                                                                      |
| `detection_height_list`                                | list[double] | Detection height for objects to be evaluated. (used for objects count only)                                                                     |
| `detection_count_purge_seconds`                        | double       | Time window for purging object detection counts.                                                                                                |
| `objects_count_window_seconds`                         | double       | Time window for keeping object detection counts. The number of object detections within this time window is stored in `detection_count_vector_` |
| `target_object.*.check_lateral_deviation`              | bool         | Whether to check lateral deviation for specific object types (car, truck, etc.).                                                                |
| `target_object.*.check_yaw_deviation`                  | bool         | Whether to check yaw deviation for specific object types (car, truck, etc.).                                                                    |
| `target_object.*.check_predicted_path_deviation`       | bool         | Whether to check predicted path deviation for specific object types (car, truck, etc.).                                                         |
| `target_object.*.check_yaw_rate`                       | bool         | Whether to check yaw rate for specific object types (car, truck, etc.).                                                                         |
| `target_object.*.check_total_objects_count`            | bool         | Whether to check total object count for specific object types (car, truck, etc.).                                                               |
| `target_object.*.check_average_objects_count`          | bool         | Whether to check average object count for specific object types (car, truck, etc.).                                                             |
| `target_object.*.check_interval_average_objects_count` | bool         | Whether to check interval average object count for specific object types (car, truck, etc.).                                                    |
| `debug_marker.*`                                       | bool         | Debugging parameters for marker visualization (history path, predicted path, etc.).                                                             |

## Assumptions / Known limits

It is assumed that the current positions of PredictedObjects are reasonably accurate.

## Future extensions / Unimplemented parts

- Increase rate in recognition per class
- Metrics for objects with strange physical behavior (e.g., going through a fence)
- Metrics for splitting objects
- Metrics for problems with objects that are normally stationary but move
- Disappearing object metrics
